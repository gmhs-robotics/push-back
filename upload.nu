#!/usr/bin/env nu

open Cargo.toml
    | upsert package.name banana-leclerc
    | upsert package.metadata.v5.slot 1
    | upsert package.metadata.v5.icon vex-coding-studio
    | to toml
    | save --force Cargo.toml

cargo v5 upload --release

open Cargo.toml
    | upsert package.name recorder
    | upsert package.metadata.v5.slot 2
    | upsert package.metadata.v5.icon code-file
    | to toml
    | save --force Cargo.toml

cargo v5 upload --release --features record
