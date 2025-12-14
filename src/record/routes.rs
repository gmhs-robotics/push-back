use std::collections::BTreeMap;

#[derive(Debug, Clone, Default)]
pub struct RouteIndex {
    /// route id -> display name
    pub map: BTreeMap<u32, String>,
}

impl RouteIndex {
    pub const INDEX_PATH: &'static str = ".routes";

    pub fn load() -> Self {
        let Ok(text) = std::fs::read_to_string(Self::INDEX_PATH) else {
            return Self::default();
        };

        let mut out = Self::default();

        for line in text.lines() {
            let (route_id, name) = match line.trim().split_once('\t') {
                Some((a, b)) => (a.trim(), b.trim()),
                None => (line, line),
            };

            if route_id.is_empty() || name.is_empty() {
                continue;
            }

            out.map.insert(
                route_id.parse::<u32>().unwrap_or_default(),
                name.to_string(),
            );
        }

        out
    }

    pub fn save(&self) -> std::io::Result<()> {
        let mut out = String::new();

        for (route_id, name) in &self.map {
            out.push_str(&route_id.to_string());
            out.push('\t');
            out.push_str(name);
            out.push('\n');
        }

        std::fs::write(Self::INDEX_PATH, out)
    }

    pub fn update(&mut self, route_id: u32, name: &str) {
        self.map.insert(route_id, name.to_string());
    }

    pub fn generate_id(&self) -> u32 {
        self.map.keys().max().unwrap_or(&0) + 1
    }
}
