// Firmware releases on GitHub, as cfcli finds the Crazyflie's: list a repo's
// latest releases and download an asset from one.
//
// The Bugslayer repositories are private, so requests carry a token when one is
// available: $GITHUB_TOKEN, $GH_TOKEN, or `gh auth token` (the GitHub CLI's
// login). Assets are fetched through the API, which is what works for private
// repositories.

use std::time::Duration;

use anyhow::{Context, Result};
use serde::Deserialize;

use crate::error::Error;

pub const OWNER: &str = "evoggy";

#[derive(Debug, Clone, Deserialize)]
pub struct Asset {
    pub id: u64,
    pub name: String,
    pub size: u64,
}

#[derive(Debug, Clone, Deserialize)]
pub struct Release {
    pub tag_name: String,
    pub draft: bool,
    pub prerelease: bool,
    pub assets: Vec<Asset>,
}

impl Release {
    /// The tag as a version (tags are bare `0.8.0`). None for tags that are not.
    pub fn version(&self) -> Option<semver::Version> {
        semver::Version::parse(&self.tag_name).ok()
    }

    pub fn asset(&self, prefix: &str, ext: &str) -> Option<&Asset> {
        self.assets.iter().find(|a| a.name.starts_with(prefix) && a.name.ends_with(ext))
    }
}

fn token() -> Option<String> {
    for var in ["GITHUB_TOKEN", "GH_TOKEN"] {
        if let Ok(t) = std::env::var(var) {
            if !t.trim().is_empty() {
                return Some(t.trim().to_string());
            }
        }
    }
    let out = std::process::Command::new("gh").args(["auth", "token"]).output().ok()?;
    let t = String::from_utf8(out.stdout).ok()?.trim().to_string();
    (out.status.success() && !t.is_empty()).then_some(t)
}

fn agent() -> ureq::Agent {
    ureq::Agent::config_builder()
        .timeout_global(Some(Duration::from_secs(60)))
        .http_status_as_error(false)
        .build()
        .into()
}

fn get(url: &str, accept: &str) -> Result<ureq::http::Response<ureq::Body>> {
    let mut req = agent()
        .get(url)
        .header("User-Agent", concat!("bugslayer/", env!("CARGO_PKG_VERSION")))
        .header("Accept", accept)
        .header("X-GitHub-Api-Version", "2022-11-28");
    let token = token();
    if let Some(t) = &token {
        req = req.header("Authorization", format!("Bearer {}", t));
    }
    let resp = req.call().map_err(|e| Error::Connection(format!("GitHub: {}", e)))?;
    match resp.status().as_u16() {
        200 => Ok(resp),
        // GitHub answers 404, not 401/403, for a private repository it won't show you.
        401 | 403 | 404 if token.is_none() => Err(Error::Connection(format!(
            "GitHub refused {} ({}). The Bugslayer repositories are private: log in with \
             `gh auth login`, or set GITHUB_TOKEN",
            url,
            resp.status()
        ))
        .into()),
        s => Err(Error::Connection(format!("GitHub: {} for {}", s, url)).into()),
    }
}

/// The repository's latest releases, newest first (drafts left out).
pub fn releases(repo: &str) -> Result<Vec<Release>> {
    let url = format!("https://api.github.com/repos/{}/{}/releases?per_page=10", OWNER, repo);
    let mut resp = get(&url, "application/vnd.github+json")?;
    let releases: Vec<Release> =
        resp.body_mut().read_json().with_context(|| format!("reading the release list of {}", repo))?;
    Ok(releases.into_iter().filter(|r| !r.draft).collect())
}

/// The newest release with a version tag; prereleases only if asked.
pub fn latest(repo: &str, prerelease: bool) -> Result<Option<Release>> {
    Ok(releases(repo)?
        .into_iter()
        .filter(|r| prerelease || !r.prerelease)
        .filter(|r| r.version().is_some())
        .max_by_key(|r| r.version()))
}

pub fn release_by_tag(repo: &str, tag: &str) -> Result<Release> {
    let tag = tag.trim_start_matches('v'); // accept v0.8.0 too
    let url = format!("https://api.github.com/repos/{}/{}/releases/tags/{}", OWNER, repo, tag);
    let mut resp = get(&url, "application/vnd.github+json")
        .map_err(|_| Error::NotFound(format!("release {} of {}", tag, repo)))?;
    Ok(resp.body_mut().read_json()?)
}

pub fn download(repo: &str, asset: &Asset) -> Result<Vec<u8>> {
    let url = format!("https://api.github.com/repos/{}/{}/releases/assets/{}", OWNER, repo, asset.id);
    let mut resp = get(&url, "application/octet-stream")?;
    let data = resp
        .body_mut()
        .with_config()
        .limit(64 << 20)
        .read_to_vec()
        .with_context(|| format!("downloading {}", asset.name))?;
    if data.len() as u64 != asset.size {
        anyhow::bail!("{}: got {} bytes, expected {}", asset.name, data.len(), asset.size);
    }
    Ok(data)
}
