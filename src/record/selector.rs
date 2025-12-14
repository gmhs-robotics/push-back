//! Simple touchscreen-based autonomous route selector.
//!
//! ![Screenshot of the `SimpleSelect` menu showing two routes](https://i.imgur.com/qM9qMsd.png)
//!
//! [`SimpleSelect`] is a barebones and lightweight autonomous selector that allows picking
//! between at most 12 autonomous routes using the V5 Brain's display and touchscreen.
//!
//! The selector provides a user interface that mimicks the appearance of other VEXos
//! dashboards, with basic support for color themes through the [`SimpleSelect::new_with_theme`]
//! function.
//!
//! # Examples
//!
//! Robot with two autonomous routes using [`SelectCompete`](crate::compete::SelectCompete).
//!
//! ```
//! use vexide::prelude::*;
//! use autons::{
//!     prelude::*,
//!     simple::{route, SimpleSelect},
//! };
//!
//! struct Robot {}
//!
//! impl Robot {
//!     async fn route_1(&mut self) {}
//!     async fn route_2(&mut self) {}
//! }
//!
//! impl SelectCompete for Robot {
//!     async fn driver(&mut self) {
//!         // ...
//!     }
//! }
//!
//! #[vexide::main]
//! async fn main(peripherals: Peripherals) {
//!     let robot = Robot {};
//!
//!     robot
//!         .compete(SimpleSelect::new(
//!             peripherals.display,
//!             [
//!                 route!("Route 1", Robot::route_1),
//!                 route!("Route 2", Robot::route_2),
//!             ],
//!         ))
//!         .await;
//! }
//! ```

use std::{cell::RefCell, rc::Rc};

use autons::{
    Selector,
    simple::{Route, SimpleSelectTheme, THEME_DARK},
};
use vexide::{
    display::{Display, Font, FontFamily, FontSize, Line, Rect, Text, TouchState},
    task::{self, Task},
    time::sleep,
};

struct SelectorState<R: 'static, const N: usize> {
    routes: [Route<R>; N],
    selection: usize,
    dirty_selection: Option<usize>,
}

/// Simple touchscreen-based autonomous route selector.
///
/// `SimpleSelect` is a barebones and lightweight autonomous selector that allows picking
/// between up to 16 autonomous routes using the V5 brain's display and touchscreen.
///
/// The selector provides a user interface that mimicks the appearance of other VEXos
/// dashboards, with basic support for color themes through the [`SimpleSelect::new_with_theme`]
/// function.
///
/// This struct implements the [`Selector`] trait and should be used with the [`SelectCompete`]
/// trait if using vexide's competition runtime.
///
/// [`SelectCompete`]: crate::compete::SelectCompete
pub struct RecorderSelect<R: 'static, const N: usize> {
    state: Rc<RefCell<SelectorState<R, N>>>,
    _task: Task<()>,
}

impl<R, const N: usize> RecorderSelect<R, N> {
    /// Creates a new selector from a [`Display`] peripheral and array of routes.
    pub fn new(display: Display, routes: [Route<R>; N]) -> Self {
        Self::new_with_theme(display, routes, THEME_DARK)
    }

    /// Creates a new selector from a [`Display`] peripheral and array of routes with a provided
    /// [custom color theme].
    ///
    /// [custom color theme]: SimpleSelectTheme
    #[allow(clippy::await_holding_refcell_ref)] // clippy is too dumb to realize we explicitly drop
    pub fn new_with_theme(
        mut display: Display,
        routes: [Route<R>; N],
        theme: SimpleSelectTheme,
    ) -> Self {
        const {
            assert!(N > 0, "SimpleSelect requires at least one route.");
            assert!(
                N <= 12,
                "SimpleSelect currently only supports up to 12 routes."
            );
        }

        let state = Rc::new(RefCell::new(SelectorState {
            routes,
            selection: 0,
            dirty_selection: None,
        }));

        Self {
            state: state.clone(),
            _task: task::spawn(async move {
                // Background
                display.fill(
                    &Rect::new(
                        [0, 0],
                        [Display::HORIZONTAL_RESOLUTION, Display::VERTICAL_RESOLUTION],
                    ),
                    theme.background_default,
                );

                // Grid lines
                Self::draw_borders(&mut display, &theme);

                {
                    let state = state.borrow();

                    for (i, route) in state.routes.iter().enumerate() {
                        Self::draw_item(
                            &mut display,
                            &theme,
                            route.name,
                            i,
                            i == state.selection,
                            false,
                        );
                    }
                }

                let mut active_item: Option<usize> = None;

                loop {
                    let mut state = state.borrow_mut();

                    let touch = display.touch_status();
                    let touch_index = ((6 * (touch.point.x / (Display::HORIZONTAL_RESOLUTION / 2)))
                        + touch.point.y / 40) as usize;

                    if matches!(touch.state, TouchState::Held | TouchState::Pressed) {
                        if active_item
                            .is_none_or(|prev_active_item| prev_active_item != touch_index)
                            && touch_index < N
                        {
                            if let Some(old_active_item) = active_item {
                                Self::draw_item(
                                    &mut display,
                                    &theme,
                                    state.routes[old_active_item].name,
                                    old_active_item,
                                    old_active_item == state.selection,
                                    false,
                                );
                            }

                            Self::draw_item(
                                &mut display,
                                &theme,
                                state.routes[touch_index].name,
                                touch_index,
                                touch_index == state.selection,
                                true,
                            );

                            active_item = Some(touch_index);
                        } else if let Some(old_active_item) = active_item {
                            if old_active_item != touch_index {
                                Self::draw_item(
                                    &mut display,
                                    &theme,
                                    state.routes[old_active_item].name,
                                    old_active_item,
                                    old_active_item == state.selection,
                                    false,
                                );

                                active_item = None;
                            }
                        }
                    } else if let Some(prev_active_item) = active_item {
                        if touch_index == prev_active_item && touch_index < N {
                            let old_selection = state.selection;

                            Self::draw_item(
                                &mut display,
                                &theme,
                                state.routes[old_selection].name,
                                old_selection,
                                false,
                                false,
                            );

                            Self::draw_item(
                                &mut display,
                                &theme,
                                state.routes[prev_active_item].name,
                                prev_active_item,
                                true,
                                false,
                            );

                            state.selection = prev_active_item;
                            active_item = None;
                        } else {
                            Self::draw_item(
                                &mut display,
                                &theme,
                                state.routes[prev_active_item].name,
                                prev_active_item,
                                false,
                                false,
                            );

                            active_item = None;
                        }
                    }

                    if let Some(dirty_selection) = state.dirty_selection {
                        Self::draw_item(
                            &mut display,
                            &theme,
                            state.routes[dirty_selection].name,
                            dirty_selection,
                            false,
                            false,
                        );

                        Self::draw_item(
                            &mut display,
                            &theme,
                            state.routes[state.selection].name,
                            state.selection,
                            true,
                            false,
                        );

                        state.dirty_selection = None;
                    }

                    drop(state);
                    sleep(Display::REFRESH_INTERVAL).await;
                }
            }),
        }
    }

    /// Programatically selects an autonomous route by index.
    pub fn select(&mut self, index: usize) {
        assert!(index < N, "Invalid route selection index.");
        let mut state = self.state.borrow_mut();
        state.dirty_selection = Some(state.selection);
        state.selection = index;
    }

    fn draw_item(
        display: &mut Display,
        theme: &SimpleSelectTheme,
        label: &str,
        index: usize,
        selected: bool,
        active: bool,
    ) {
        let (background_color, text_color) = match (selected, active) {
            (false, false) => (theme.background_default, theme.text_default),
            (false, true) => (theme.background_active, theme.text_active),
            (true, false) => (theme.background_selected, theme.text_selected),
            (true, true) => (theme.background_selected_active, theme.text_selected_active),
        };

        display.fill(
            &Rect::from_dimensions(
                [
                    if index <= 5 {
                        0
                    } else {
                        Display::HORIZONTAL_RESOLUTION / 2
                    },
                    (index % 6) as i16 * 40,
                ],
                238,
                38,
            ),
            background_color,
        );

        display.draw_text(
            &Text::from_string(
                label,
                Font::new(FontSize::MEDIUM, FontFamily::Proportional),
                [
                    if index <= 5 {
                        8
                    } else {
                        Display::HORIZONTAL_RESOLUTION / 2 + 8
                    },
                    ((index % 6) as i16 * 40 + 6),
                ],
            ),
            text_color,
            None,
        );
    }

    fn draw_borders(display: &mut Display, theme: &SimpleSelectTheme) {
        // Vertical gridline
        display.fill(
            &Line::new(
                [Display::HORIZONTAL_RESOLUTION / 2 - 1, 0],
                [
                    Display::HORIZONTAL_RESOLUTION / 2 - 1,
                    Display::VERTICAL_RESOLUTION,
                ],
            ),
            theme.border,
        );

        // Horizontal gridline
        for n in 1..=5 {
            display.fill(
                &Line::new(
                    [0, n * 40 - 1],
                    [Display::HORIZONTAL_RESOLUTION, n * 40 - 1],
                ),
                theme.border,
            );
        }
    }
}

impl<R, const N: usize> Selector<R> for RecorderSelect<R, N> {
    async fn run(&self, robot: &mut R) {
        {
            let state = self.state.borrow();
            (state.routes[state.selection].callback)(robot)
        }
        .await;
    }
}
