use core::{future::Future, pin::Pin};
use std::{cell::RefCell, rc::Rc, time::{Duration, Instant}};

use autons::{simple::{SimpleSelectTheme, THEME_DARK}, Selector};
use vexide::{
    display::{Display, Font, FontFamily, FontSize, Line, Rect, Text, TouchState},
    task::{self, Task},
    time::sleep,
};

#[cfg(feature = "record")]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RecordTarget {
    Off,
    New,
    Overwrite(u32),
}

#[cfg(feature = "record")]
#[derive(Debug, Clone)]
pub struct RecordOption {
    pub label: String,
    pub target: RecordTarget,
}

#[cfg(not(feature = "record"))]
#[derive(Debug, Clone)]
pub struct PlaybackChoice {
    pub label: String,
    pub route_id: Option<u32>,
}

#[cfg(feature = "record")]
type RecordHook<R> = fn(&mut R, RecordTarget);
#[cfg(not(feature = "record"))]
type PlaybackFn<R> = for<'a> fn(
    &'a mut R,
    Option<u32>,
) -> Pin<Box<dyn Future<Output = ()> + 'a>>;

#[derive(Debug, Clone)]
struct StatusMessage {
    text: String,
    set_at: Instant,
}

#[cfg(feature = "record")]
struct SelectorState<R: 'static> {
    record_options: Vec<RecordOption>,
    record_callback: RecordHook<R>,
    record_selection: usize,
    active_row: Option<usize>,
    dirty_rows: Vec<usize>,
    status: Option<StatusMessage>,
    status_dirty: bool,
}

#[cfg(not(feature = "record"))]
struct SelectorState<R: 'static> {
    playback_choices: Vec<PlaybackChoice>,
    playback_callback: PlaybackFn<R>,
    playback_selection: usize,
    active_row: Option<usize>,
    dirty_rows: Vec<usize>,
    status: Option<StatusMessage>,
    status_dirty: bool,
}

pub struct RecorderSelect<R: 'static> {
    state: Rc<RefCell<SelectorState<R>>>,
    _task: Task<()>,
}

#[derive(Clone)]
pub struct StatusHandle<R: 'static> {
    state: Rc<RefCell<SelectorState<R>>>,
}

impl<R> RecorderSelect<R> {
    const STATUS_HEIGHT: i16 = 24;

    #[cfg(feature = "record")]
    pub fn new(
        display: Display,
        record_options: Vec<RecordOption>,
        default_record_selection: usize,
        record_callback: RecordHook<R>,
    ) -> Self {
        Self::new_with_theme(display, record_options, default_record_selection, record_callback, THEME_DARK)
    }

    #[cfg(feature = "record")]
    #[allow(clippy::await_holding_refcell_ref)]
    pub fn new_with_theme(
        mut display: Display,
        record_options: Vec<RecordOption>,
        default_record_selection: usize,
        record_callback: RecordHook<R>,
        theme: SimpleSelectTheme,
    ) -> Self {
        assert!(
            default_record_selection < record_options.len(),
            "Invalid default record selection index",
        );

        let rows = record_options.len().max(1);
        let cell_height = Self::cell_height(rows);

        let state = Rc::new(RefCell::new(SelectorState {
            record_options,
            record_callback,
            record_selection: default_record_selection,
            active_row: None,
            dirty_rows: Vec::new(),
            status: None,
            status_dirty: true,
        }));

        Self {
            state: state.clone(),
            _task: task::spawn(async move {
                display.fill(
                    &Rect::new(
                        [0, 0],
                        [Display::HORIZONTAL_RESOLUTION, Display::VERTICAL_RESOLUTION],
                    ),
                    theme.background_default,
                );

                Self::draw_borders(&mut display, &theme, cell_height);

                {
                    let state = state.borrow();
                    for row_index in 0..state.record_options.len() {
                        Self::draw_item(
                            &mut display,
                            &theme,
                            &state.record_options[row_index].label,
                            row_index,
                            row_index == state.record_selection,
                            false,
                            cell_height,
                        );
                    }
                }

                Self::draw_status(&mut display, &theme, None);

                loop {
                    let touch = display.touch_status();
                    let touched_index = match touch.state {
                        TouchState::Pressed | TouchState::Held if touch.point.y < Self::list_height() => {
                            let row_index: usize =
                                (touch.point.y / cell_height).try_into().unwrap_or_default();
                            Some(row_index)
                        }
                        _ => None,
                    };

                    let mut state = state.borrow_mut();

                    let prev_selection = state.record_selection;
                    let prev_active = state.active_row;

                    if let Some(row_index) = touched_index {
                        if row_index < state.record_options.len() {
                            state.record_selection = row_index;
                        }
                    }

                    state.active_row = touched_index.filter(|row| *row < state.record_options.len());

                    let current_selection = state.record_selection;

                    if prev_selection != current_selection {
                        state.dirty_rows.extend([prev_selection, current_selection]);
                    }

                    if prev_active != state.active_row {
                        if let Some(prev) = prev_active {
                            state.dirty_rows.push(prev);
                        }

                        if let Some(current) = state.active_row {
                            state.dirty_rows.push(current);
                        }
                    }

                    let mut status_text = state.status.as_ref().map(|s| s.text.clone());
                    if let Some(status) = &state.status {
                        if Instant::now().saturating_duration_since(status.set_at)
                            >= Self::status_duration()
                        {
                            state.status = None;
                            status_text = None;
                            state.status_dirty = true;
                        }
                    }

                    let redraw_status = if state.status_dirty {
                        state.status_dirty = false;
                        true
                    } else {
                        false
                    };

                    let record_selection = state.record_selection;
                    let active_row = state.active_row;
                    let dirty_rows = core::mem::take(&mut state.dirty_rows);
                    let redraw_rows: Vec<(usize, String)> = dirty_rows
                        .into_iter()
                        .filter_map(|index| {
                            state
                                .record_options
                                .get(index)
                                .map(|option| (index, option.label.clone()))
                        })
                        .collect();

                    drop(state);

                    for (row_index, label) in redraw_rows
                        .into_iter()
                        .filter(|(row_index, _)| *row_index < rows)
                    {
                        Self::draw_item(
                            &mut display,
                            &theme,
                            &label,
                            row_index,
                            row_index == record_selection,
                            active_row == Some(row_index),
                            cell_height,
                        );
                    }

                    if redraw_status {
                        Self::draw_status(&mut display, &theme, status_text.as_deref());
                    }

                    sleep(Display::REFRESH_INTERVAL).await;
                }
            }),
        }
    }

    #[cfg(not(feature = "record"))]
    pub fn new(
        display: Display,
        playback_choices: Vec<PlaybackChoice>,
        playback_callback: PlaybackFn<R>,
    ) -> Self {
        Self::new_with_theme(display, playback_choices, playback_callback, THEME_DARK)
    }

    #[cfg(not(feature = "record"))]
    #[allow(clippy::await_holding_refcell_ref)]
    pub fn new_with_theme(
        mut display: Display,
        playback_choices: Vec<PlaybackChoice>,
        playback_callback: PlaybackFn<R>,
        theme: SimpleSelectTheme,
    ) -> Self {
        assert!(
            !playback_choices.is_empty(),
            "RecorderSelect requires at least one playback route.",
        );

        let rows = playback_choices.len().max(1);
        let cell_height = Self::cell_height(rows);

        let state = Rc::new(RefCell::new(SelectorState {
            playback_choices,
            playback_callback,
            playback_selection: 0,
            active_row: None,
            dirty_rows: Vec::new(),
            status: None,
            status_dirty: true,
        }));

        Self {
            state: state.clone(),
            _task: task::spawn(async move {
                display.fill(
                    &Rect::new(
                        [0, 0],
                        [Display::HORIZONTAL_RESOLUTION, Display::VERTICAL_RESOLUTION],
                    ),
                    theme.background_default,
                );

                Self::draw_borders(&mut display, &theme, cell_height);

                {
                    let state = state.borrow();
                    for row_index in 0..state.playback_choices.len() {
                        Self::draw_item(
                            &mut display,
                            &theme,
                            &state.playback_choices[row_index].label,
                            row_index,
                            row_index == state.playback_selection,
                            false,
                            cell_height,
                        );
                    }
                }

                Self::draw_status(&mut display, &theme, None);

                loop {
                    let touch = display.touch_status();
                    let touched_index = match touch.state {
                        TouchState::Pressed | TouchState::Held if touch.point.y < Self::list_height() => {
                            let row_index: usize =
                                (touch.point.y / cell_height).try_into().unwrap_or_default();
                            Some(row_index)
                        }
                        _ => None,
                    };

                    let mut state = state.borrow_mut();

                    let prev_selection = state.playback_selection;
                    let prev_active = state.active_row;

                    if let Some(row_index) = touched_index {
                        if row_index < state.playback_choices.len() {
                            state.playback_selection = row_index;
                        }
                    }

                    state.active_row = touched_index.filter(|row| *row < state.playback_choices.len());

                    let current_selection = state.playback_selection;

                    if prev_selection != current_selection {
                        state.dirty_rows.extend([prev_selection, current_selection]);
                    }

                    if prev_active != state.active_row {
                        if let Some(prev) = prev_active {
                            state.dirty_rows.push(prev);
                        }

                        if let Some(current) = state.active_row {
                            state.dirty_rows.push(current);
                        }
                    }

                    let mut status_text = state.status.as_ref().map(|s| s.text.clone());
                    if let Some(status) = &state.status {
                        if Instant::now().saturating_duration_since(status.set_at)
                            >= Self::status_duration()
                        {
                            state.status = None;
                            status_text = None;
                            state.status_dirty = true;
                        }
                    }

                    let redraw_status = if state.status_dirty {
                        state.status_dirty = false;
                        true
                    } else {
                        false
                    };

                    let playback_selection = state.playback_selection;
                    let active_row = state.active_row;
                    let dirty_rows = core::mem::take(&mut state.dirty_rows);
                    let redraw_rows: Vec<(usize, String)> = dirty_rows
                        .into_iter()
                        .filter_map(|index| {
                            state
                                .playback_choices
                                .get(index)
                                .map(|choice| (index, choice.label.clone()))
                        })
                        .collect();

                    drop(state);

                    for (row_index, label) in redraw_rows
                        .into_iter()
                        .filter(|(row_index, _)| *row_index < rows)
                    {
                        Self::draw_item(
                            &mut display,
                            &theme,
                            &label,
                            row_index,
                            row_index == playback_selection,
                            active_row == Some(row_index),
                            cell_height,
                        );
                    }

                    if redraw_status {
                        Self::draw_status(&mut display, &theme, status_text.as_deref());
                    }

                    sleep(Display::REFRESH_INTERVAL).await;
                }
            }),
        }
    }

    #[cfg(feature = "record")]
    pub fn record_target(&self) -> RecordTarget {
        self.state.borrow().record_options[self.state.borrow().record_selection].target
    }

    #[cfg(not(feature = "record"))]
    pub fn selected_playback(&self) -> Option<u32> {
        self.state.borrow().playback_choices[self.state.borrow().playback_selection].route_id
    }

    pub fn status_handle(&self) -> StatusHandle<R> {
        StatusHandle {
            state: self.state.clone(),
        }
    }

    fn draw_item(
        display: &mut Display,
        theme: &SimpleSelectTheme,
        label: &str,
        row: usize,
        selected: bool,
        active: bool,
        cell_height: i16,
    ) {
        let (background_color, text_color) = match (selected, active) {
            (false, false) => (theme.background_default, theme.text_default),
            (false, true) => (theme.background_active, theme.text_active),
            (true, false) => (theme.background_selected, theme.text_selected),
            (true, true) => (theme.background_selected_active, theme.text_selected_active),
        };

        let width: u16 = (Display::HORIZONTAL_RESOLUTION - 2)
            .try_into()
            .unwrap_or_default();
        let height: u16 = cell_height.saturating_sub(2).try_into().unwrap_or_default();

        display.fill(
            &Rect::from_dimensions([0, row as i16 * cell_height], width, height),
            background_color,
        );

        display.draw_text(
            &Text::from_string(
                label,
                Font::new(FontSize::MEDIUM, FontFamily::Proportional),
                [8, row as i16 * cell_height + 6],
            ),
            text_color,
            None,
        );
    }

    fn draw_borders(display: &mut Display, theme: &SimpleSelectTheme, cell_height: i16) {
        let rows = Self::list_height() / cell_height;
        for n in 1..=rows {
            display.fill(
                &Line::new([0, n * cell_height - 1], [Display::HORIZONTAL_RESOLUTION, n * cell_height - 1]),
                theme.border,
            );
        }
    }

    fn draw_status(display: &mut Display, theme: &SimpleSelectTheme, text: Option<&str>) {
        let y_start = Self::list_height();
        let height: u16 = Self::STATUS_HEIGHT.try_into().unwrap_or_default();

        display.fill(
            &Rect::from_dimensions([0, y_start], Display::HORIZONTAL_RESOLUTION as u16, height),
            theme.background_default,
        );

        if let Some(text) = text {
            display.draw_text(
                &Text::from_string(
                    text,
                    Font::new(FontSize::MEDIUM, FontFamily::Proportional),
                    [8, y_start + 4],
                ),
                theme.text_default,
                None,
            );
        }
    }

    fn list_height() -> i16 {
        Display::VERTICAL_RESOLUTION - Self::STATUS_HEIGHT
    }

    fn cell_height(rows: usize) -> i16 {
        (Self::list_height() / rows.max(1) as i16).max(1)
    }

    fn status_duration() -> Duration {
        Duration::from_secs(2)
    }
}

#[cfg(feature = "record")]
impl<R> Selector<R> for RecorderSelect<R> {
    async fn run(&self, robot: &mut R) {
        let callback = {
            let state = self.state.borrow();
            let record_target = state.record_options[state.record_selection].target;
            (state.record_callback, record_target)
        };

        (callback.0)(robot, callback.1);
    }
}

#[cfg(not(feature = "record"))]
impl<R> Selector<R> for RecorderSelect<R> {
    async fn run(&self, robot: &mut R) {
        let (callback, route_id) = {
            let state = self.state.borrow();
            (
                state.playback_callback,
                state.playback_choices[state.playback_selection].route_id,
            )
        };

        {
            let mut state = self.state.borrow_mut();
            let selection = state.playback_selection;
            state.dirty_rows.push(selection);
        }

        (callback)(robot, route_id).await;
    }
}

impl<R> StatusHandle<R> {
    pub fn show_status(&self, text: impl Into<String>) {
        let mut state = self.state.borrow_mut();
        state.status = Some(StatusMessage {
            text: text.into(),
            set_at: Instant::now(),
        });
        state.status_dirty = true;
    }
}
