use std::env;
use std::path::PathBuf;

fn main() {
    let manifest_dir = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").unwrap());

    // Add the manifest dir to the linker search path so `riscv-rt`'s
    // `link.x` finds our `memory.x`.
    println!("cargo:rustc-link-search={}", manifest_dir.display());

    // Tell the linker to use `link.x` from `riscv-rt` (which then INCLUDEs
    // `memory.x` we provide). Self-contained — works without a project-level
    // `.cargo/config.toml` at the consumer side.
    println!("cargo:rustc-link-arg=-Tlink.x");

    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=memory.x");
}
