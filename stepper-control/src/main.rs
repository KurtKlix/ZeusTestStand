#![windows_subsystem = "windows"]

use eframe::egui::{self, Color32, RichText, ScrollArea};
use serialport::SerialPort;
use std::io::{self, BufRead, BufReader, Write};
use std::process::Command;
use std::sync::{
    atomic::{AtomicBool, Ordering},
    mpsc::{self, Receiver},
    Arc,
};
use std::time::{Duration, Instant};

#[derive(PartialEq, Default)]
enum HomingState {
    #[default]
    NotHomed,
    Homing,
    Homed,
}

#[derive(PartialEq, Default)]
enum Tab {
    #[default]
    Control,
    Config,
}

#[derive(Clone, Default)]
struct MacroDef {
    name: String,
    label: String,
    /// Newline-separated G-code lines (semicolons used in the config file on disk).
    gcode: String,
}

struct StepperApp {
    // Connection
    port_name: String,
    connected: bool,
    serial_port: Option<Box<dyn SerialPort>>,
    log: Vec<String>,
    rx: Option<Receiver<String>>,
    reader_running: Option<Arc<AtomicBool>>,

    // Control tab
    distance: f32,
    speed: f32,
    min_pos: f32,
    max_pos: f32,
    current_pos: f32,
    homing_state: HomingState,
    endstop_since: Option<Instant>,
    endstop_y_min: String,
    endstop_y_max: String,

    // UI state
    active_tab: Tab,

    // Config tab — runtime settings (EEPROM)
    config_steps_y: f32,
    config_max_feedrate_y: f32,
    config_max_accel_y: f32,
    config_motor_current_y: f32,
    reading_config: bool,

    // Firmware management
    firmware_dir: String,
    compile_y_home_dir: i32,
    compile_y_min_pos: f32,
    compile_y_max_pos: f32,
    compile_y_min_endstop_override: bool,
    compile_y_min_endstop_pin: String,
    compile_y_max_endstop_override: bool,
    compile_y_max_endstop_pin: String,
    compiling: bool,
    compile_rx: Option<Receiver<String>>,

    // Macros
    macros: Vec<MacroDef>,
    editing_macro: bool,
    macro_edit_idx: Option<usize>,
    macro_edit_name: String,
    macro_edit_label: String,
    macro_edit_gcode: String,
}

impl Default for StepperApp {
    fn default() -> Self {
        Self {
            port_name: "COM3".to_string(),
            connected: false,
            serial_port: None,
            log: Vec::new(),
            rx: None,
            reader_running: None,
            distance: 10.0,
            speed: 1000.0,
            min_pos: 5.0,
            max_pos: 500.0,
            current_pos: 0.0,
            homing_state: HomingState::NotHomed,
            endstop_since: None,
            endstop_y_min: String::new(),
            endstop_y_max: String::new(),
            active_tab: Tab::Control,
            config_steps_y: 80.0,
            config_max_feedrate_y: 500.0,
            config_max_accel_y: 500.0,
            config_motor_current_y: 800.0,
            reading_config: false,
            firmware_dir: String::new(),
            compile_y_home_dir: 1,
            compile_y_min_pos: 0.0,
            compile_y_max_pos: 500.0,
            compile_y_min_endstop_override: false,
            compile_y_min_endstop_pin: String::new(),
            compile_y_max_endstop_override: false,
            compile_y_max_endstop_pin: String::new(),
            compiling: false,
            compile_rx: None,
            macros: Vec::new(),
            editing_macro: false,
            macro_edit_idx: None,
            macro_edit_name: String::new(),
            macro_edit_label: String::new(),
            macro_edit_gcode: String::new(),
        }
    }
}

fn parse_axis_value(line: &str, axis: char) -> Option<f32> {
    let needle = format!(" {axis}");
    let pos = line.find(&needle)?;
    let after = &line[pos + 2..];
    let end = after
        .find(|c: char| matches!(c, ' ' | '\t' | '\r' | '\n'))
        .unwrap_or(after.len());
    after[..end].parse().ok()
}

impl StepperApp {
    // -----------------------------------------------------------------------
    // Serial connection
    // -----------------------------------------------------------------------

    fn connect(&mut self) {
        match serialport::new(&self.port_name, 115200)
            .timeout(Duration::from_millis(10))
            .open()
        {
            Ok(port) => match port.try_clone() {
                Ok(reader_port) => {
                    let (tx, rx) = mpsc::channel();
                    let running = Arc::new(AtomicBool::new(true));
                    let running_clone = running.clone();

                    std::thread::spawn(move || {
                        let mut reader = BufReader::new(reader_port);
                        let mut line = String::new();
                        while running_clone.load(Ordering::Relaxed) {
                            line.clear();
                            match reader.read_line(&mut line) {
                                Ok(0) => break,
                                Ok(_) => {
                                    let trimmed = line.trim().to_string();
                                    if !trimmed.is_empty() && tx.send(trimmed).is_err() {
                                        break;
                                    }
                                }
                                Err(e)
                                    if e.kind() == io::ErrorKind::TimedOut
                                        || e.kind() == io::ErrorKind::WouldBlock =>
                                {
                                    continue;
                                }
                                Err(_) => break,
                            }
                        }
                    });

                    self.serial_port = Some(port);
                    self.rx = Some(rx);
                    self.reader_running = Some(running);
                    self.connected = true;
                    self.homing_state = HomingState::NotHomed;
                    self.current_pos = 0.0;
                    self.reading_config = false;
                    self.push_log("Connected.".to_string());
                    self.send_raw("G91");
                }
                Err(e) => self.push_log(format!("Failed to clone port: {e}")),
            },
            Err(e) => self.push_log(format!("Connect error: {e}")),
        }
    }

    fn disconnect(&mut self) {
        if let Some(running) = self.reader_running.take() {
            running.store(false, Ordering::Relaxed);
        }
        self.serial_port = None;
        self.rx = None;
        self.connected = false;
        self.homing_state = HomingState::NotHomed;
        self.reading_config = false;
        self.push_log("Disconnected.".to_string());
    }

    fn send_raw(&mut self, cmd: &str) {
        if let Some(port) = &mut self.serial_port {
            let msg = format!("{cmd}\n");
            match port.write_all(msg.as_bytes()) {
                Ok(_) => {
                    let _ = port.flush();
                    self.log.push(format!("> {cmd}"));
                }
                Err(e) => {
                    self.log.push(format!("Send error: {e}"));
                    self.connected = false;
                }
            }
        }
    }

    // -----------------------------------------------------------------------
    // Motion
    // -----------------------------------------------------------------------

    fn home_y(&mut self) {
        // Requires #define Y_HOME_DIR 1 in Marlin Configuration.h for +Y homing.
        self.homing_state = HomingState::Homing;
        self.send_raw("G28 Y");
    }

    fn move_y(&mut self, direction: f32) {
        let dist = self.distance * direction;
        let speed = self.speed;
        match self.homing_state {
            HomingState::Homed => {
                let target = (self.current_pos + dist).clamp(self.min_pos, self.max_pos);
                let actual = target - self.current_pos;
                if actual.abs() < 0.01 {
                    self.push_log(format!(
                        "At position limit ({:.1} mm) — move ignored.",
                        self.current_pos
                    ));
                    return;
                }
                self.current_pos = target;
                self.send_raw(&format!("G1 Y{actual:.2} F{speed:.0}"));
            }
            _ => {
                self.send_raw(&format!("G1 Y{dist:.2} F{speed:.0}"));
            }
        }
    }

    // -----------------------------------------------------------------------
    // Endstop
    // -----------------------------------------------------------------------

    fn query_endstop(&mut self) {
        self.endstop_since = Some(Instant::now());
        self.endstop_y_min.clear();
        self.endstop_y_max.clear();
        self.send_raw("M119");
    }

    // -----------------------------------------------------------------------
    // EEPROM config
    // -----------------------------------------------------------------------

    fn read_config(&mut self) {
        self.reading_config = true;
        self.send_raw("M503");
    }

    fn apply_config(&mut self) {
        let steps = self.config_steps_y;
        let feedrate = self.config_max_feedrate_y;
        let accel = self.config_max_accel_y;
        let current = self.config_motor_current_y as u32;
        self.send_raw(&format!("M92 Y{steps:.2}"));
        self.send_raw(&format!("M203 Y{feedrate:.2}"));
        self.send_raw(&format!("M201 Y{accel:.2}"));
        self.send_raw(&format!("M906 Y{current}"));
        self.push_log(
            "Changes applied. Click \"Save to EEPROM\" to persist across reboots.".to_string(),
        );
    }

    // -----------------------------------------------------------------------
    // Macros
    // -----------------------------------------------------------------------

    fn run_macro(&mut self, idx: usize) {
        if let Some(m) = self.macros.get(idx).cloned() {
            for cmd in m.gcode.lines() {
                let cmd = cmd.trim();
                if !cmd.is_empty() && !cmd.starts_with('#') {
                    self.send_raw(cmd);
                }
            }
        }
    }

    // -----------------------------------------------------------------------
    // Config file I/O  (zeus_config.cfg)
    // -----------------------------------------------------------------------

    fn config_file_path(&self) -> std::path::PathBuf {
        std::path::PathBuf::from(&self.firmware_dir).join("zeus_config.cfg")
    }

    fn zeus_h_path(&self) -> std::path::PathBuf {
        std::path::PathBuf::from(&self.firmware_dir)
            .join("Marlin")
            .join("Configuration_Zeus.h")
    }

    fn save_config(&mut self) {
        let path = self.config_file_path();
        let mut content = format!(
            "# Zeus Test Stand Configuration\n\
             # Managed by Stepper Control app\n\
             \n\
             # Runtime settings (applied via G-code; also stored in EEPROM)\n\
             steps_per_mm={}\n\
             max_feedrate={}\n\
             max_accel={}\n\
             motor_current={}\n\
             \n\
             # Compile-time settings (require Compile & Flash)\n\
             y_home_dir={}\n\
             y_min_pos={}\n\
             y_max_pos={}\n\
             y_min_endstop_override={}\n\
             y_min_endstop_pin={}\n\
             y_max_endstop_override={}\n\
             y_max_endstop_pin={}\n",
            self.config_steps_y,
            self.config_max_feedrate_y,
            self.config_max_accel_y,
            self.config_motor_current_y as u32,
            self.compile_y_home_dir,
            self.compile_y_min_pos,
            self.compile_y_max_pos,
            self.compile_y_min_endstop_override,
            self.compile_y_min_endstop_pin,
            self.compile_y_max_endstop_override,
            self.compile_y_max_endstop_pin,
        );

        for m in &self.macros {
            // Store gcode as semicolon-separated so it stays on one line.
            let gcode_flat: String = m
                .gcode
                .lines()
                .map(|l| l.trim())
                .filter(|l| !l.is_empty())
                .collect::<Vec<_>>()
                .join(";");
            content.push_str(&format!(
                "\n[macro:{}]\nlabel={}\ngcode={}\n",
                m.name, m.label, gcode_flat
            ));
        }

        match std::fs::write(&path, &content) {
            Ok(_) => {
                self.push_log(format!("Saved {}", path.display()));
                self.save_settings();
            }
            Err(e) => self.push_log(format!("Save error: {e}")),
        }
    }

    fn load_config(&mut self) {
        let path = self.config_file_path();
        match std::fs::read_to_string(&path) {
            Ok(text) => {
                self.macros.clear();
                let mut current_macro: Option<MacroDef> = None;

                for raw in text.lines() {
                    let line = raw.trim();
                    if line.starts_with('#') || line.is_empty() {
                        continue;
                    }

                    // Section header [macro:NAME]
                    if line.starts_with('[') && line.ends_with(']') {
                        if let Some(m) = current_macro.take() {
                            self.macros.push(m);
                        }
                        let inner = &line[1..line.len() - 1];
                        if let Some(name) = inner.strip_prefix("macro:") {
                            current_macro = Some(MacroDef {
                                name: name.trim().to_string(),
                                label: name.trim().to_string(),
                                gcode: String::new(),
                            });
                        }
                        continue;
                    }

                    if let Some((k, v)) = line.split_once('=') {
                        let k = k.trim();
                        let v = v.trim();
                        if let Some(ref mut m) = current_macro {
                            match k {
                                "label" => m.label = v.to_string(),
                                "gcode" => {
                                    m.gcode = v.replace(';', "\n");
                                }
                                _ => {}
                            }
                        } else {
                            match k {
                                "steps_per_mm" => {
                                    if let Ok(n) = v.parse() {
                                        self.config_steps_y = n;
                                    }
                                }
                                "max_feedrate" => {
                                    if let Ok(n) = v.parse() {
                                        self.config_max_feedrate_y = n;
                                    }
                                }
                                "max_accel" => {
                                    if let Ok(n) = v.parse() {
                                        self.config_max_accel_y = n;
                                    }
                                }
                                "motor_current" => {
                                    if let Ok(n) = v.parse::<f32>() {
                                        self.config_motor_current_y = n;
                                    }
                                }
                                "y_home_dir" => {
                                    if let Ok(n) = v.parse() {
                                        self.compile_y_home_dir = n;
                                    }
                                }
                                "y_min_pos" => {
                                    if let Ok(n) = v.parse() {
                                        self.compile_y_min_pos = n;
                                    }
                                }
                                "y_max_pos" => {
                                    if let Ok(n) = v.parse() {
                                        self.compile_y_max_pos = n;
                                    }
                                }
                                "y_min_endstop_override" => {
                                    self.compile_y_min_endstop_override = v == "true";
                                }
                                "y_min_endstop_pin" => {
                                    self.compile_y_min_endstop_pin = v.to_string();
                                }
                                "y_max_endstop_override" => {
                                    self.compile_y_max_endstop_override = v == "true";
                                }
                                "y_max_endstop_pin" => {
                                    self.compile_y_max_endstop_pin = v.to_string();
                                }
                                _ => {}
                            }
                        }
                    }
                }
                if let Some(m) = current_macro {
                    self.macros.push(m);
                }
                self.push_log(format!("Loaded {}", path.display()));
            }
            Err(e) => self.push_log(format!("Load error: {e}")),
        }
    }

    // -----------------------------------------------------------------------
    // Firmware file helpers
    // -----------------------------------------------------------------------

    fn write_zeus_h(&mut self) {
        let path = self.zeus_h_path();
        let mut content = format!(
            "// Configuration_Zeus.h — Auto-generated by Stepper Control\n\
             // Add to the end of Configuration.h:\n\
             //   #include \"Configuration_Zeus.h\"\n\
             // Do not edit manually.\n\
             \n\
             #ifdef Y_HOME_DIR\n\
             #  undef Y_HOME_DIR\n\
             #endif\n\
             #define Y_HOME_DIR {}\n\
             \n\
             #ifdef Y_MIN_POS\n\
             #  undef Y_MIN_POS\n\
             #endif\n\
             #define Y_MIN_POS {}\n\
             \n\
             #ifdef Y_MAX_POS\n\
             #  undef Y_MAX_POS\n\
             #endif\n\
             #define Y_MAX_POS {}\n",
            self.compile_y_home_dir, self.compile_y_min_pos, self.compile_y_max_pos,
        );

        let min_pin = self.compile_y_min_endstop_pin.trim().to_string();
        let max_pin = self.compile_y_max_endstop_pin.trim().to_string();

        if self.compile_y_min_endstop_override && !min_pin.is_empty() {
            content.push_str(&format!(
                "\n#ifdef Y_MIN_ENDSTOP_PIN\n\
                 #  undef Y_MIN_ENDSTOP_PIN\n\
                 #endif\n\
                 #define Y_MIN_ENDSTOP_PIN {min_pin}\n"
            ));
        }
        if self.compile_y_max_endstop_override && !max_pin.is_empty() {
            content.push_str(&format!(
                "\n#ifdef Y_MAX_ENDSTOP_PIN\n\
                 #  undef Y_MAX_ENDSTOP_PIN\n\
                 #endif\n\
                 #define Y_MAX_ENDSTOP_PIN {max_pin}\n"
            ));
        }

        match std::fs::write(&path, &content) {
            Ok(_) => self.push_log(format!("Wrote {}", path.display())),
            Err(e) => self.push_log(format!("Write error: {e}")),
        }
    }

    /// Ensures Configuration.h contains `#include "Configuration_Zeus.h"`.
    /// Inserts it just before the final `#endif` if not already present.
    fn patch_configuration_h(&mut self) {
        let path = std::path::PathBuf::from(&self.firmware_dir)
            .join("Marlin")
            .join("Configuration.h");

        let text = match std::fs::read_to_string(&path) {
            Ok(t) => t,
            Err(e) => {
                self.push_log(format!("Cannot read Configuration.h: {e}"));
                return;
            }
        };

        let include_line = "#include \"Configuration_Zeus.h\"";
        if text.contains(include_line) {
            self.push_log("Configuration.h already includes Configuration_Zeus.h.".to_string());
            return;
        }

        // Insert before the very last #endif in the file.
        let patched = if let Some(pos) = text.rfind("#endif") {
            format!("{}\n{}\n\n{}", &text[..pos], include_line, &text[pos..])
        } else {
            format!("{}\n{}\n", text, include_line)
        };

        match std::fs::write(&path, patched) {
            Ok(_) => self.push_log(
                "Patched Configuration.h — added #include \"Configuration_Zeus.h\".".to_string(),
            ),
            Err(e) => self.push_log(format!("Cannot patch Configuration.h: {e}")),
        }
    }

    fn compile_and_flash(&mut self) {
        if self.firmware_dir.trim().is_empty() {
            self.push_log("Set firmware directory before compiling.".to_string());
            return;
        }
        self.save_config();
        self.write_zeus_h();
        self.patch_configuration_h();

        let dir = self.firmware_dir.clone();
        let (tx, rx) = mpsc::channel::<String>();
        self.compile_rx = Some(rx);
        self.compiling = true;

        std::thread::spawn(move || {
            let _ = tx.send("[pio] Starting PlatformIO compile & upload...".to_string());
            match Command::new("pio")
                .args(["run", "--target", "upload"])
                .current_dir(&dir)
                .output()
            {
                Err(e) => {
                    let _ = tx.send(format!("[pio] Failed to launch pio: {e}"));
                    let _ = tx
                        .send("[pio] Is PlatformIO Core installed and in PATH?".to_string());
                }
                Ok(out) => {
                    for line in String::from_utf8_lossy(&out.stdout).lines() {
                        if !line.trim().is_empty() {
                            let _ = tx.send(format!("[pio] {line}"));
                        }
                    }
                    for line in String::from_utf8_lossy(&out.stderr).lines() {
                        if !line.trim().is_empty() {
                            let _ = tx.send(format!("[pio] {line}"));
                        }
                    }
                    if out.status.success() {
                        let _ = tx.send("[pio] ✓ Flash complete.".to_string());
                    } else {
                        let _ =
                            tx.send(format!("[pio] ✗ Build failed (exit {}).", out.status));
                    }
                }
            }
            let _ = tx.send("\x00DONE".to_string());
        });
    }

    // -----------------------------------------------------------------------
    // Polling
    // -----------------------------------------------------------------------

    fn poll_compile(&mut self) {
        if self.compile_rx.is_none() {
            return;
        }
        let msgs: Vec<String> = self
            .compile_rx
            .as_ref()
            .map(|rx| rx.try_iter().collect())
            .unwrap_or_default();
        for msg in msgs {
            if msg == "\x00DONE" {
                self.compiling = false;
                self.compile_rx = None;
            } else {
                self.push_log(msg);
            }
        }
    }

    fn push_log(&mut self, msg: String) {
        self.log.push(msg);
        if self.log.len() > 500 {
            self.log.drain(0..self.log.len() - 500);
        }
    }

    fn poll_serial(&mut self) {
        let messages: Vec<String> = self
            .rx
            .as_ref()
            .map(|rx| rx.try_iter().collect())
            .unwrap_or_default();

        for msg in messages {
            if self.reading_config {
                if msg.contains("M92") {
                    if let Some(v) = parse_axis_value(&msg, 'Y') {
                        self.config_steps_y = v;
                    }
                }
                if msg.contains("M203") {
                    if let Some(v) = parse_axis_value(&msg, 'Y') {
                        self.config_max_feedrate_y = v;
                    }
                }
                if msg.contains("M201") {
                    if let Some(v) = parse_axis_value(&msg, 'Y') {
                        self.config_max_accel_y = v;
                    }
                }
                if msg.contains("M906") {
                    if let Some(v) = parse_axis_value(&msg, 'Y') {
                        self.config_motor_current_y = v;
                    }
                }
            }

            // Parse M119 endstop lines unconditionally — Marlin sends `ok`
            // immediately (before the data lines), so gating on endstop_pending
            // would clear the flag before the y_min/y_max lines ever arrive.
            {
                let lower = msg.to_lowercase();
                if let Some(rest) = lower.strip_prefix("y_min:") {
                    self.endstop_y_min = rest.trim().to_string();
                } else if let Some(rest) = lower.strip_prefix("y_max:") {
                    self.endstop_y_max = rest.trim().to_string();
                }
            }

            self.push_log(format!("< {msg}"));
            let is_ok = msg.trim_start().starts_with("ok");

            if is_ok && self.reading_config {
                self.reading_config = false;
                self.push_log("Config loaded from board.".to_string());
            }
            if is_ok && self.homing_state == HomingState::Homing {
                self.homing_state = HomingState::Homed;
                self.current_pos = self.max_pos;
                self.send_raw("G91");
                self.push_log(format!(
                    "Homed. Position set to {:.1} mm. Relative moves active.",
                    self.max_pos
                ));
            }
        }
    }

    // -----------------------------------------------------------------------
    // Persistent settings  (firmware_dir saved next to the exe)
    // -----------------------------------------------------------------------

    fn settings_path() -> std::path::PathBuf {
        std::env::current_exe()
            .ok()
            .and_then(|p| p.parent().map(|d| d.join("zeus_settings.cfg")))
            .unwrap_or_else(|| std::path::PathBuf::from("zeus_settings.cfg"))
    }

    fn save_settings(&self) {
        let path = Self::settings_path();
        let _ = std::fs::write(
            &path,
            format!(
                "# Zeus Test Stand settings — auto-generated\nfirmware_dir={}\n",
                self.firmware_dir
            ),
        );
    }

    /// Called once at startup. Loads the saved firmware_dir and, if the
    /// corresponding zeus_config.cfg exists, loads that too so all values
    /// are restored without any manual steps.
    fn load_settings(&mut self) {
        let path = Self::settings_path();
        if let Ok(text) = std::fs::read_to_string(&path) {
            for line in text.lines() {
                let line = line.trim();
                if line.starts_with('#') || line.is_empty() {
                    continue;
                }
                if let Some((k, v)) = line.split_once('=') {
                    if k.trim() == "firmware_dir" && !v.trim().is_empty() {
                        self.firmware_dir = v.trim().to_string();
                    }
                }
            }
        }
        // Silently load zeus_config.cfg if the firmware dir is known.
        if !self.firmware_dir.is_empty() && self.config_file_path().exists() {
            self.load_config();
        }
    }
}

// ---------------------------------------------------------------------------
// UI
// ---------------------------------------------------------------------------

impl eframe::App for StepperApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.poll_serial();
        self.poll_compile();

        // Serial log pinned to the bottom so it's always visible.
        egui::TopBottomPanel::bottom("log_panel")
            .resizable(true)
            .min_height(80.0)
            .default_height(160.0)
            .show(ctx, |ui| {
                ui.label(RichText::new("Serial Log").strong());
                ScrollArea::vertical()
                    .stick_to_bottom(true)
                    .show(ui, |ui| {
                        for entry in &self.log {
                            let color = if entry.starts_with('>') {
                                Color32::LIGHT_BLUE
                            } else if entry.starts_with('<') {
                                Color32::LIGHT_GREEN
                            } else if entry.starts_with("[pio]") {
                                Color32::from_rgb(255, 210, 100)
                            } else if entry.to_lowercase().contains("error")
                                || entry.contains('✗')
                            {
                                Color32::RED
                            } else {
                                Color32::GRAY
                            };
                            ui.label(RichText::new(entry).monospace().color(color));
                        }
                    });
            });

        egui::CentralPanel::default().show(ctx, |ui| {
            ui.heading("Stepper Motor Control");
            ui.label(RichText::new("Y Axis — YM Connector").weak());
            ui.separator();

            // --- Connection ---
            ui.group(|ui| {
                ui.horizontal(|ui| {
                    ui.label("COM Port:");
                    ui.add_enabled(
                        !self.connected,
                        egui::TextEdit::singleline(&mut self.port_name).desired_width(70.0),
                    );
                    if self.connected {
                        if ui.button("Disconnect").clicked() {
                            self.disconnect();
                        }
                        ui.label(RichText::new("● Connected").color(Color32::GREEN));
                    } else {
                        if ui.button("Connect").clicked() {
                            self.connect();
                        }
                        ui.label(RichText::new("○ Disconnected").color(Color32::GRAY));
                    }
                });
            });

            ui.add_space(6.0);

            // --- Tab bar ---
            ui.horizontal(|ui| {
                ui.selectable_value(&mut self.active_tab, Tab::Control, "  Motor Control  ");
                ui.selectable_value(&mut self.active_tab, Tab::Config, "  Firmware Config  ");
            });
            ui.separator();

            // --- Scrollable tab content ---
            ScrollArea::vertical()
                .id_salt("tab_scroll")
                .show(ui, |ui| {
                    match self.active_tab {
                        Tab::Control => self.show_control_tab(ui),
                        Tab::Config => self.show_config_tab(ui),
                    }
                });
        });

        ctx.request_repaint_after(Duration::from_millis(50));
    }
}

impl StepperApp {
    fn show_control_tab(&mut self, ui: &mut egui::Ui) {
        // --- Homing & endstop ---
        ui.group(|ui| {
            let busy = self.homing_state == HomingState::Homing;

            ui.horizontal(|ui| {
                ui.add_enabled_ui(self.connected && !busy, |ui| {
                    if ui.button("Home Y  ⌂").clicked() {
                        self.home_y();
                    }
                });
                match self.homing_state {
                    HomingState::NotHomed => {
                        ui.label(RichText::new("Not homed").color(Color32::YELLOW));
                    }
                    HomingState::Homing => {
                        ui.label(RichText::new("Homing…").color(Color32::YELLOW));
                    }
                    HomingState::Homed => {
                        ui.label(
                            RichText::new(format!("Position:  {:.2} mm", self.current_pos))
                                .color(Color32::LIGHT_GREEN),
                        );
                    }
                }
            });

            ui.add_space(2.0);

            // Endstop query row — pending expires after 2 s regardless of ok/no-ok
            let endstop_pending = self
                .endstop_since
                .map(|t| t.elapsed() < Duration::from_secs(2))
                .unwrap_or(false);
            ui.horizontal(|ui| {
                let can_query = self.connected && !busy && !endstop_pending;
                ui.add_enabled_ui(can_query, |ui| {
                    if ui.button("Query Endstop").clicked() {
                        self.query_endstop();
                    }
                });
                if endstop_pending {
                    ui.label(RichText::new("Querying…").color(Color32::YELLOW));
                } else {
                    let es_color = |s: &str| {
                        if s.eq_ignore_ascii_case("triggered") {
                            Color32::RED
                        } else if s == "open" {
                            Color32::LIGHT_GREEN
                        } else {
                            Color32::GRAY
                        }
                    };
                    if !self.endstop_y_min.is_empty() || !self.endstop_y_max.is_empty() {
                        ui.label(
                            RichText::new(format!("y_min: {}", self.endstop_y_min))
                                .color(es_color(&self.endstop_y_min)),
                        );
                        ui.label(
                            RichText::new(format!("y_max: {}", self.endstop_y_max))
                                .color(es_color(&self.endstop_y_max)),
                        );
                    }
                }
            });

            ui.add_space(4.0);

            ui.horizontal(|ui| {
                ui.label("Min limit:");
                let max_for_min = self.max_pos - 1.0;
                ui.add(
                    egui::DragValue::new(&mut self.min_pos)
                        .speed(0.5)
                        .range(0.0..=max_for_min)
                        .suffix(" mm"),
                );
                ui.add_space(12.0);
                ui.label("Max limit:");
                let min_for_max = self.min_pos + 1.0;
                ui.add(
                    egui::DragValue::new(&mut self.max_pos)
                        .speed(1.0)
                        .range(min_for_max..=10000.0)
                        .suffix(" mm"),
                );
            });
        });

        ui.add_space(8.0);

        // --- Distance / speed / movement ---
        ui.group(|ui| {
            ui.horizontal(|ui| {
                ui.label("Distance:");
                ui.add(
                    egui::DragValue::new(&mut self.distance)
                        .speed(0.5)
                        .range(0.1..=500.0)
                        .suffix(" mm"),
                );
            });
            ui.horizontal(|ui| {
                ui.label("Speed:      ");
                ui.add(
                    egui::DragValue::new(&mut self.speed)
                        .speed(10.0)
                        .range(100.0..=10000.0)
                        .suffix(" mm/min"),
                );
            });

            ui.add_space(8.0);

            let can_move = self.connected && self.homing_state != HomingState::Homing;
            ui.add_enabled_ui(can_move, |ui| {
                ui.horizontal(|ui| {
                    if ui
                        .add_sized([130.0, 44.0], egui::Button::new("◄  Backward"))
                        .clicked()
                    {
                        self.move_y(-1.0);
                    }
                    ui.add_space(8.0);
                    if ui
                        .add_sized([130.0, 44.0], egui::Button::new("Forward  ►"))
                        .clicked()
                    {
                        self.move_y(1.0);
                    }
                });
            });
        });

        // --- Macros (only rendered when macros are defined) ---
        if !self.macros.is_empty() {
            ui.add_space(8.0);
            ui.group(|ui| {
                ui.label(RichText::new("Macros").strong());
                ui.add_space(4.0);

                let mut to_run: Option<usize> = None;
                let can_run = self.connected && self.homing_state != HomingState::Homing;

                ui.add_enabled_ui(can_run, |ui| {
                    ui.horizontal_wrapped(|ui| {
                        for (i, m) in self.macros.iter().enumerate() {
                            if ui
                                .add_sized(
                                    [140.0, 36.0],
                                    egui::Button::new(RichText::new(&m.label)),
                                )
                                .clicked()
                            {
                                to_run = Some(i);
                            }
                        }
                    });
                });

                if let Some(idx) = to_run {
                    self.run_macro(idx);
                }
            });
        }
    }

    fn show_config_tab(&mut self, ui: &mut egui::Ui) {
        // --- Configuration file ---
        ui.group(|ui| {
            ui.label(RichText::new("Configuration File").strong());
            ui.add_space(4.0);
            ui.horizontal(|ui| {
                ui.label("Firmware dir:");
                ui.add(
                    egui::TextEdit::singleline(&mut self.firmware_dir)
                        .hint_text("C:\\path\\to\\Marlin-firmware")
                        .desired_width(300.0),
                );
            });
            if !self.firmware_dir.is_empty() {
                ui.label(
                    RichText::new(format!(
                        "Config:  {}\\zeus_config.cfg\n\
                         Header:  {}\\Marlin\\Configuration_Zeus.h",
                        self.firmware_dir, self.firmware_dir
                    ))
                    .weak()
                    .small(),
                );
            }
            ui.add_space(4.0);
            ui.add_enabled_ui(!self.firmware_dir.trim().is_empty(), |ui| {
                ui.horizontal(|ui| {
                    if ui.button("Load Config").clicked() {
                        self.load_config();
                    }
                    if ui.button("Save Config").clicked() {
                        self.save_config();
                    }
                });
            });
        });

        ui.add_space(8.0);

        // --- Runtime settings (EEPROM) ---
        ui.group(|ui| {
            ui.label(RichText::new("Runtime Settings — stored in EEPROM").strong());
            ui.add_space(6.0);

            egui::Grid::new("config_grid")
                .num_columns(3)
                .spacing([16.0, 8.0])
                .striped(true)
                .show(ui, |ui| {
                    ui.label("Y Steps/mm");
                    ui.add(
                        egui::DragValue::new(&mut self.config_steps_y)
                            .speed(0.1)
                            .range(1.0..=10000.0)
                            .suffix(" steps/mm"),
                    );
                    ui.label(RichText::new("M92 Y  — pulses per mm of travel").weak().small());
                    ui.end_row();

                    ui.label("Max Feedrate");
                    ui.add(
                        egui::DragValue::new(&mut self.config_max_feedrate_y)
                            .speed(1.0)
                            .range(1.0..=5000.0)
                            .suffix(" mm/s"),
                    );
                    ui.label(RichText::new("M203 Y  — hard speed ceiling").weak().small());
                    ui.end_row();

                    ui.label("Max Acceleration");
                    ui.add(
                        egui::DragValue::new(&mut self.config_max_accel_y)
                            .speed(1.0)
                            .range(1.0..=50000.0)
                            .suffix(" mm/s²"),
                    );
                    ui.label(RichText::new("M201 Y  — ramp rate").weak().small());
                    ui.end_row();

                    ui.label("Motor Current");
                    ui.add(
                        egui::DragValue::new(&mut self.config_motor_current_y)
                            .speed(5.0)
                            .range(100.0..=1200.0)
                            .suffix(" mA"),
                    );
                    ui.label(RichText::new("M906 Y  — TMC2209 RMS current").weak().small());
                    ui.end_row();
                });

            ui.add_space(8.0);

            let busy = !self.connected
                || self.reading_config
                || self.homing_state == HomingState::Homing;

            ui.horizontal(|ui| {
                ui.add_enabled_ui(!busy, |ui| {
                    if ui.button("Read from Board").clicked() {
                        self.read_config();
                    }
                    if ui.button("Apply Changes").clicked() {
                        self.apply_config();
                    }
                    if ui.button("Save to EEPROM  💾").clicked() {
                        self.send_raw("M500");
                        self.push_log("Settings saved to EEPROM.".to_string());
                    }
                    if ui.button("Reset Defaults").clicked() {
                        self.send_raw("M502");
                        self.push_log(
                            "EEPROM reset to firmware defaults. Click Read from Board to refresh."
                                .to_string(),
                        );
                    }
                });
                if self.reading_config {
                    ui.label(RichText::new("Reading…").color(Color32::YELLOW));
                }
            });
        });

        ui.add_space(8.0);

        // --- Compile-time settings ---
        ui.group(|ui| {
            ui.label(
                RichText::new("Compile-time Settings — require Compile & Flash").strong(),
            );
            ui.add_space(6.0);

            egui::Grid::new("ct_grid")
                .num_columns(2)
                .spacing([16.0, 8.0])
                .striped(true)
                .show(ui, |ui| {
                    ui.label("Y_HOME_DIR");
                    ui.horizontal(|ui| {
                        ui.radio_value(&mut self.compile_y_home_dir, -1, "-1  (toward min)");
                        ui.radio_value(&mut self.compile_y_home_dir, 1, "+1  (toward max)");
                    });
                    ui.end_row();

                    ui.label("Y_MIN_POS");
                    ui.add(
                        egui::DragValue::new(&mut self.compile_y_min_pos)
                            .speed(1.0)
                            .range(-500.0..=500.0)
                            .suffix(" mm"),
                    );
                    ui.end_row();

                    ui.label("Y_MAX_POS");
                    ui.add(
                        egui::DragValue::new(&mut self.compile_y_max_pos)
                            .speed(1.0)
                            .range(0.0..=10000.0)
                            .suffix(" mm"),
                    );
                    ui.end_row();

                    ui.label("Y_MIN_ENDSTOP_PIN");
                    ui.horizontal(|ui| {
                        ui.checkbox(&mut self.compile_y_min_endstop_override, "override");
                        ui.add_enabled(
                            self.compile_y_min_endstop_override,
                            egui::TextEdit::singleline(&mut self.compile_y_min_endstop_pin)
                                .hint_text("e.g. PC7 or 3")
                                .desired_width(80.0),
                        );
                    });
                    ui.end_row();

                    ui.label("Y_MAX_ENDSTOP_PIN");
                    ui.horizontal(|ui| {
                        ui.checkbox(&mut self.compile_y_max_endstop_override, "override");
                        ui.add_enabled(
                            self.compile_y_max_endstop_override,
                            egui::TextEdit::singleline(&mut self.compile_y_max_endstop_pin)
                                .hint_text("e.g. PC8 or 14")
                                .desired_width(80.0),
                        );
                    });
                    ui.end_row();
                });

            ui.add_space(6.0);
            ui.label(
                RichText::new(
                    "\"Write Header\" and \"Compile & Flash\" both auto-patch Configuration.h \
                     to add the #include if it is not already there.",
                )
                .weak()
                .small(),
            );
            ui.add_space(8.0);

            let can_flash = !self.firmware_dir.trim().is_empty() && !self.compiling;
            ui.horizontal(|ui| {
                ui.add_enabled_ui(can_flash, |ui| {
                    if ui.button("Write Header Only").clicked() {
                        self.write_zeus_h();
                        self.patch_configuration_h();
                    }
                    if ui.button("Compile & Flash  ⚡").clicked() {
                        self.compile_and_flash();
                    }
                });
                if self.compiling {
                    ui.label(
                        RichText::new("Compiling… see log below").color(Color32::YELLOW),
                    );
                }
            });
        });

        ui.add_space(8.0);

        // --- Macros ---
        ui.group(|ui| {
            ui.horizontal(|ui| {
                ui.label(RichText::new("Macros").strong());
                if ui.small_button("  + Add  ").clicked() && !self.editing_macro {
                    self.macro_edit_idx = None;
                    self.macro_edit_name.clear();
                    self.macro_edit_label.clear();
                    self.macro_edit_gcode.clear();
                    self.editing_macro = true;
                }
            });

            ui.add_space(4.0);

            // Macro list
            let mut to_delete: Option<usize> = None;
            let mut to_edit: Option<usize> = None;
            let mut to_run: Option<usize> = None;

            egui::Grid::new("macro_list")
                .num_columns(4)
                .spacing([8.0, 4.0])
                .striped(true)
                .show(ui, |ui| {
                    for (i, m) in self.macros.iter().enumerate() {
                        ui.label(&m.label);
                        ui.label(RichText::new(&m.name).weak().small());
                        ui.horizontal(|ui| {
                            if ui.small_button("▶ Run").clicked() {
                                to_run = Some(i);
                            }
                            if ui.small_button("Edit").clicked() && !self.editing_macro {
                                to_edit = Some(i);
                            }
                            if ui.small_button("✕").clicked() && !self.editing_macro {
                                to_delete = Some(i);
                            }
                        });
                        ui.end_row();
                    }
                });

            if let Some(idx) = to_run {
                self.run_macro(idx);
            }
            if let Some(idx) = to_delete {
                self.macros.remove(idx);
            }
            if let Some(idx) = to_edit {
                self.macro_edit_idx = Some(idx);
                self.macro_edit_name = self.macros[idx].name.clone();
                self.macro_edit_label = self.macros[idx].label.clone();
                self.macro_edit_gcode = self.macros[idx].gcode.clone();
                self.editing_macro = true;
            }

            // Inline editor
            if self.editing_macro {
                ui.add_space(8.0);
                ui.separator();
                ui.label(
                    RichText::new(if self.macro_edit_idx.is_some() {
                        "Edit Macro"
                    } else {
                        "New Macro"
                    })
                    .strong(),
                );
                ui.add_space(4.0);

                egui::Grid::new("macro_editor")
                    .num_columns(2)
                    .spacing([8.0, 6.0])
                    .show(ui, |ui| {
                        ui.label("Name");
                        ui.add(
                            egui::TextEdit::singleline(&mut self.macro_edit_name)
                                .hint_text("move_to_load  (no spaces)")
                                .desired_width(200.0),
                        );
                        ui.end_row();

                        ui.label("Button label");
                        ui.add(
                            egui::TextEdit::singleline(&mut self.macro_edit_label)
                                .hint_text("Move to Load Position")
                                .desired_width(200.0),
                        );
                        ui.end_row();

                        ui.label("G-code");
                        ui.add(
                            egui::TextEdit::multiline(&mut self.macro_edit_gcode)
                                .hint_text("G1 Y100 F500\nG4 P200")
                                .desired_rows(5)
                                .desired_width(280.0),
                        );
                        ui.end_row();
                    });

                ui.add_space(6.0);
                ui.horizontal(|ui| {
                    let name_ok = !self.macro_edit_name.trim().is_empty()
                        && !self.macro_edit_name.contains(' ');
                    ui.add_enabled_ui(name_ok, |ui| {
                        if ui.button("Save Macro").clicked() {
                            let m = MacroDef {
                                name: self.macro_edit_name.trim().to_string(),
                                label: if self.macro_edit_label.trim().is_empty() {
                                    self.macro_edit_name.trim().to_string()
                                } else {
                                    self.macro_edit_label.trim().to_string()
                                },
                                gcode: self.macro_edit_gcode.clone(),
                            };
                            if let Some(idx) = self.macro_edit_idx {
                                self.macros[idx] = m;
                            } else {
                                self.macros.push(m);
                            }
                            self.editing_macro = false;
                        }
                    });
                    if ui.button("Cancel").clicked() {
                        self.editing_macro = false;
                    }
                    if !self.macro_edit_name.trim().is_empty()
                        && self.macro_edit_name.contains(' ')
                    {
                        ui.label(
                            RichText::new("Name cannot contain spaces").color(Color32::RED),
                        );
                    }
                });
            }
        });
    }
}

fn main() -> eframe::Result<()> {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([600.0, 800.0])
            .with_min_inner_size([480.0, 520.0])
            .with_resizable(true)
            .with_title("Stepper Control"),
        ..Default::default()
    };

    eframe::run_native(
        "Stepper Control",
        options,
        Box::new(|_cc| {
            let mut app = StepperApp::default();
            app.load_settings();
            Ok(Box::new(app))
        }),
    )
}
