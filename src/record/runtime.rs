use std::time::Instant;

use autons::prelude::{SelectCompete, SelectCompeteExt};
use vexide::{prelude::*, time::sleep};

use super::{
    routes::RouteIndex,
    selector::{PlaybackChoice, RecordOption, RecordTarget, RecorderSelect, StatusHandle},
};
use crate::record::frame::{Frameable, Recordable, Recording};

#[allow(dead_code)]
#[derive(Debug, Default)]
pub struct RouteRecorder<F: Frameable> {
    target: Option<RecordTarget>,
    current: Option<Recording<F>>,
    last_frame_time: Option<Instant>,
}

#[allow(dead_code)]
impl<F: Frameable> RouteRecorder<F> {
    pub fn new() -> Self {
        let mut recorder = Self::default();
        recorder.set_target(RecordTarget::Off);
        recorder
    }

    pub fn set_target(&mut self, target: RecordTarget) {
        println!("{target:?}");

        self.target = match target {
            RecordTarget::Off => None,
            other => Some(other),
        };

        match self.target {
            Some(_) => {
                self.current = Some(Recording::default());
                self.last_frame_time = None;
            }
            None => {
                self.current = None;
                self.last_frame_time = None;
            }
        }
    }

    pub fn target(&self) -> RecordTarget {
        self.target.unwrap_or_default()
    }

    pub fn is_recording(&self) -> bool {
        self.target.is_some()
    }

    pub fn push_frame(&mut self, frame: F) {
        if let Some(recording) = &mut self.current {
            let now = Instant::now();
            let delta = if let Some(last) = self.last_frame_time.replace(now) {
                now.saturating_duration_since(last)
            } else {
                Default::default()
            };

            recording.push_timed(delta, frame);
        }
    }

    pub fn finish(&mut self) -> Option<(RecordTarget, Recording<F>)> {
        if !self.current.as_ref()?.frames.is_empty() {
            return None;
        }

        println!("2");
        let target = self.target.take()?;

        println!("3");
        let recording = self.current.take()?;

        println!("4?");
        self.last_frame_time = None;

        Some((target, recording))
    }
}

#[allow(dead_code)]
pub struct RecordingAutonomous<R: Recordable + 'static> {
    pub robot: R,
    pub index: RouteIndex,
    recorder: RouteRecorder<R::Frame>,
    status: StatusHandle<RecordOption>,
}

#[allow(dead_code)]
impl<R: Recordable + 'static> RecordingAutonomous<R> {
    pub async fn compete(robot: R, display: Display) -> ! {
        let index = RouteIndex::load();

        let record_options: Vec<RecordOption> = [
            RecordOption {
                label: "Record Off".to_owned(),
                target: RecordTarget::Off,
            },
            RecordOption {
                label: "Record New Route".to_owned(),
                target: RecordTarget::New,
            },
        ]
        .into_iter()
        .chain(index.entries().iter().map(|entry| RecordOption {
            label: format!("Record over {}", entry.display_name),
            target: RecordTarget::Overwrite(entry.id),
        }))
        .collect();

        let selector = RecorderSelect::new(display, record_options, 0, Self::arm_recording);

        let status = selector.status_handle();

        let recorder = RouteRecorder::new();

        Self {
            robot,
            index,
            recorder,
            status,
        }
        .compete(selector)
        .await;
    }

    async fn save_recording(&mut self, target: RecordTarget, recording: Recording<R::Frame>) {
        let Some(route_id) = (match target {
            RecordTarget::Off => None,
            RecordTarget::New => Some(self.index.next_id()),
            RecordTarget::Overwrite(id) => Some(id),
        }) else {
            return;
        };

        let display_name = self.index.display_name(route_id);
        let path = RouteIndex::path_for(route_id);

        if recording.save(&path).is_ok() {
            self.index.update(route_id, &display_name);
            let _ = self.index.save();
            self.status.show_status(format!("Saved {display_name}"));
        }
    }

    fn arm_recording(
        &mut self,
        option: RecordOption,
    ) -> core::pin::Pin<Box<dyn core::future::Future<Output = ()> + '_>> {
        self.recorder.set_target(option.target);

        if let RecordTarget::Off = option.target {
            self.status.show_status("Recording off");
        } else {
            self.status.show_status(format!("Armed: {}", option.label));
        }

        Box::pin(async {})
    }
}

#[allow(dead_code)]
pub struct PlaybackAutonomous<R: Recordable + 'static> {
    pub robot: R,
    pub index: RouteIndex,
    active_route: Option<u32>,
    status: StatusHandle<PlaybackChoice>,
}

#[allow(dead_code)]
impl<R: Recordable + 'static> PlaybackAutonomous<R> {
    pub async fn compete(robot: R, display: Display) -> ! {
        let index = RouteIndex::load();

        let mut playback_choices = vec![PlaybackChoice {
            label: "Disable".to_string(),
            route_id: None,
        }];

        playback_choices.extend(index.entries().into_iter().map(|entry| PlaybackChoice {
            label: entry.display_name,
            route_id: Some(entry.id),
        }));

        let selector = RecorderSelect::new(display, playback_choices, 0, Self::play_selected);
        let status = selector.status_handle();

        Self {
            robot,
            index,
            active_route: None,
            status,
        }
        .compete(selector)
        .await;
    }

    fn play_selected(
        &mut self,
        choice: PlaybackChoice,
    ) -> core::pin::Pin<Box<dyn core::future::Future<Output = ()> + '_>> {
        self.active_route = choice.route_id;

        Box::pin(async move {
            let Some(route_id) = choice.route_id else {
                return;
            };

            let path = RouteIndex::path_for(route_id);
            let display_name = self.index.display_name(route_id);

            if let Ok(recording) = Recording::load(&path) {
                self.status.show_status(format!("Playing {display_name}"));
                recording.playback(&mut self.robot).await;
            } else {
                self.status
                    .show_status(format!("Missing route {display_name}"));
            }
        })
    }
}

impl<R: Recordable + 'static> SelectCompete for RecordingAutonomous<R> {
    async fn driver(&mut self) {
        loop {
            let frame = self.robot.get_new_frame().await;
            if self.recorder.is_recording() {
                self.recorder.push_frame(frame.clone());
            }

            self.robot.transform_to_frame(&frame).await;
            sleep(R::UPDATE_INTERVAL).await;
        }
    }

    async fn disabled(&mut self) {
        if let Some((target, recording)) = self.recorder.finish() {
            println!("suaved");
            self.save_recording(target, recording).await;
        }
    }
}

impl<R: Recordable + 'static> SelectCompete for PlaybackAutonomous<R> {
    async fn driver(&mut self) {
        loop {
            let frame = self.robot.get_new_frame().await;
            self.robot.transform_to_frame(&frame).await;
            sleep(R::UPDATE_INTERVAL).await;
        }
    }
}
