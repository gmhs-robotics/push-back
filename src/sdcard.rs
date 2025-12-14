pub fn is_sdcard_inserted() -> bool {
    unsafe { vex_sdk::vexFileDriveStatus(0) }
}
