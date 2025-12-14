use std::{collections::BTreeMap, path::PathBuf};

#[derive(Debug, Clone, Default)]
pub struct RouteIndex {
    /// route id -> display name
    pub map: BTreeMap<u32, String>,
}

#[derive(Debug, Clone)]
pub struct RouteEntry {
    pub id: u32,
    pub display_name: String,
    pub path: PathBuf,
}

#[derive(Debug, Clone, Default)]
pub struct RouteLibrary {
    pub index: RouteIndex,
    pub entries: Vec<RouteEntry>,
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

impl RouteLibrary {
    pub const ROUTE_EXTENSION: &'static str = "route";

    pub fn load() -> Self {
        let index = RouteIndex::load();
        let mut entries: Vec<RouteEntry> = std::fs::read_dir(".")
            .into_iter()
            .flatten()
            .flatten()
            .filter_map(|entry| {
                let path = entry.path();
                let Some(file_stem) = path.file_stem()?.to_str() else {
                    return None;
                };

                if path.extension()?.to_str()? != Self::ROUTE_EXTENSION {
                    return None;
                }

                let id: u32 = file_stem.parse().ok()?;
                let display_name = index
                    .map
                    .get(&id)
                    .cloned()
                    .unwrap_or_else(|| id.to_string());

                Some(RouteEntry {
                    id,
                    display_name,
                    path,
                })
            })
            .collect();

        for (id, display_name) in &index.map {
            if entries.iter().any(|entry| entry.id == *id) {
                continue;
            }

            entries.push(RouteEntry {
                id: *id,
                display_name: display_name.clone(),
                path: Self::path_for(*id),
            });
        }

        entries.sort_by_key(|entry| entry.id);

        Self { index, entries }
    }

    pub fn next_id(&self) -> u32 {
        self.entries
            .iter()
            .map(|entry| entry.id)
            .max()
            .map(|max| max + 1)
            .unwrap_or(1)
    }

    pub fn path_for(id: u32) -> PathBuf {
        PathBuf::from(format!("/{id}.{}", Self::ROUTE_EXTENSION))
    }

    pub fn ensure_entry_name(&mut self, id: u32, name: &str) {
        self.index.update(id, name);
        if let Some(entry) = self.entries.iter_mut().find(|entry| entry.id == id) {
            entry.display_name = name.to_string();
        } else {
            self.entries.push(RouteEntry {
                id,
                display_name: name.to_string(),
                path: Self::path_for(id),
            });
        }
    }

    pub fn persist_index(&self) -> std::io::Result<()> {
        self.index.save()
    }
}
