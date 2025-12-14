use autons::prelude::SelectCompete;
use vexide::{prelude::*, time::sleep};

use crate::record::frame::{Frameable, Recordable, Recording};

use super::routes::RouteLibrary;
#[cfg(feature = "record")]
use super::selector::{RecordOption, RecordTarget, RecorderSelect, StatusHandle};
#[cfg(not(feature = "record"))]
use super::selector::{PlaybackChoice, RecorderSelect};

#[cfg(feature = "record")]
#[derive(Debug, Default)]
pub struct RouteRecorder<F: Frameable> {
    target: Option<RecordTarget>,
    current: Option<Recording<F>>,
    last_frame_time: Option<std::time::Instant>,
}

#[cfg(feature = "record")]
impl<F: Frameable> RouteRecorder<F> {
    pub fn new(default_target: RecordTarget) -> Self {
        let mut recorder = Self::default();
        recorder.set_target(default_target);
        recorder
    }

    pub fn set_target(&mut self, target: RecordTarget) {
        self.target = match target {
            RecordTarget::Off => None,
            _ => Some(target),
        };

        if self.target.is_some() {
            self.current.get_or_insert_with(Recording::default);
            self.last_frame_time = None;
        }
    }

    pub fn target(&self) -> RecordTarget {
        self.target.unwrap_or(RecordTarget::Off)
    }

    pub fn is_recording(&self) -> bool {
        self.target.is_some()
    }

    pub fn push_frame(&mut self, frame: F) {
        if let Some(recording) = &mut self.current {
            let now = std::time::Instant::now();
            let delta = if let Some(last) = self.last_frame_time.replace(now) {
                now.saturating_duration_since(last)
            } else {
                Default::default()
            };

            recording.push_timed(delta, frame);
        }
    }

    pub fn finish(&mut self) -> Option<(RecordTarget, Recording<F>)> {
        let target = self.target.take()?;
        let recording = self.current.take()?;
        self.last_frame_time = None;
        Some((target, recording))
    }
}

#[cfg(feature = "record")]
pub struct AutonomousRecorder<R: Recordable + 'static> {
    pub robot: R,
    pub library: RouteLibrary,
    recorder: RouteRecorder<R::Frame>,
    driver_tick: std::time::Duration,
    status: StatusHandle<Self>,
}

#[cfg(feature = "record")]
impl<R: Recordable + 'static> AutonomousRecorder<R> {
    pub fn new(
        robot: R,
        display: Display,
        driver_tick: std::time::Duration,
        default_record_target: RecordTarget,
    ) -> (Self, RecorderSelect<Self>) {
        let library = RouteLibrary::load();

        let mut record_options = vec![RecordOption {
            label: "Record Off".to_string(),
            target: RecordTarget::Off,
        }];

        record_options.push(RecordOption {
            label: "Record New Route".to_string(),
            target: RecordTarget::New,
        });

        for entry in &library.entries {
            record_options.push(RecordOption {
                label: format!("Record over {}", entry.display_name),
                target: RecordTarget::Overwrite(entry.id),
            });
        }

        let default_record_selection = record_options
            .iter()
            .position(|option| option.target == default_record_target)
            .unwrap_or(0);

        let selector = RecorderSelect::new(
            display,
            record_options,
            default_record_selection,
            Self::arm_recording,
        );

        let status = selector.status_handle();

        let mut recorder = RouteRecorder::new(selector.record_target());
        recorder.set_target(default_record_target);

        (
            Self {
                robot,
                library,
                recorder,
                driver_tick,
                status,
            },
            selector,
        )
    }

    async fn save_recording(&mut self, target: RecordTarget, recording: Recording<R::Frame>) {
        let Some(route_id) = (match target {
            RecordTarget::Off => None,
            RecordTarget::New => Some(self.library.next_id()),
            RecordTarget::Overwrite(id) => Some(id),
        }) else {
            return;
        };

        let display_name = self
            .library
            .index
            .map
            .get(&route_id)
            .cloned()
            .unwrap_or_else(|| route_id.to_string());

        let path = RouteLibrary::path_for(route_id);
        if recording.save(&path).is_ok() {
            self.library.ensure_entry_name(route_id, &display_name);
            let _ = self.library.persist_index();
            self.status
                .show_status(format!("Saved {}", display_name));
        }
    }

    fn arm_recording(&mut self, target: RecordTarget) {
        self.recorder.set_target(target);
    }
}

#[cfg(feature = "record")]
impl<R: Recordable + 'static> Recordable for AutonomousRecorder<R> {
    type Frame = R::Frame;

    async fn transform_to_frame(&mut self, frame: &Self::Frame) {
        self.robot.transform_to_frame(frame).await;
    }

    async fn get_new_frame(&self) -> Self::Frame {
        self.robot.get_new_frame().await
    }
}

#[cfg(feature = "record")]
impl<R: Recordable + 'static> SelectCompete for AutonomousRecorder<R> {
    async fn driver(&mut self) {
        loop {
            let frame = self.robot.get_new_frame().await;
            if self.recorder.is_recording() {
                self.recorder.push_frame(frame.clone());
            }

            self.robot.transform_to_frame(&frame).await;
            sleep(self.driver_tick).await;
        }
    }

    async fn disabled(&mut self) {
        if let Some((target, recording)) = self.recorder.finish() {
            self.save_recording(target, recording).await;
        }
    }
}

#[cfg(not(feature = "record"))]
pub struct AutonomousRecorder<R: Recordable + 'static> {
    pub robot: R,
    pub library: RouteLibrary,
    active_route: Option<u32>,
    driver_tick: std::time::Duration,
}

#[cfg(not(feature = "record"))]
impl<R: Recordable + 'static> AutonomousRecorder<R> {
    pub fn new(
        robot: R,
        display: Display,
        driver_tick: std::time::Duration,
    ) -> (Self, RecorderSelect<Self>) {
        let library = RouteLibrary::load();

        let mut playback_choices = vec![PlaybackChoice {
            label: "Disable".to_string(),
            route_id: None,
        }];

        playback_choices.extend(
            library
                .entries
                .iter()
                .map(|entry| PlaybackChoice {
                    label: entry.display_name.clone(),
                    route_id: Some(entry.id),
                }),
        );

        let selector = RecorderSelect::new(display, playback_choices, Self::play_selected);

        (
            Self {
                robot,
                library,
                active_route: None,
                driver_tick,
            },
            selector,
        )
    }

    fn play_selected(
        &mut self,
        route_id: Option<u32>,
    ) -> core::pin::Pin<Box<dyn core::future::Future<Output = ()> + '_>> {
        self.active_route = route_id;

        Box::pin(async move {
            let Some(route_id) = route_id else {
                return;
            };

            if let Some(entry) = self.library.entries.iter().find(|entry| entry.id == route_id) {
                if let Ok(recording) = Recording::load(&entry.path) {
                    recording.playback(&mut self.robot).await;
                }
            }
        })
    }
}

#[cfg(not(feature = "record"))]
impl<R: Recordable + 'static> Recordable for AutonomousRecorder<R> {
    type Frame = R::Frame;

    async fn transform_to_frame(&mut self, frame: &Self::Frame) {
        self.robot.transform_to_frame(frame).await;
    }

    async fn get_new_frame(&self) -> Self::Frame {
        self.robot.get_new_frame().await
    }
}

#[cfg(not(feature = "record"))]
impl<R: Recordable + 'static> SelectCompete for AutonomousRecorder<R> {
    async fn driver(&mut self) {
        loop {
            let frame = self.robot.get_new_frame().await;
            self.robot.transform_to_frame(&frame).await;
            sleep(self.driver_tick).await;
        }
    }

    async fn disabled(&mut self) {}
}
