{
  rustPlatform,
  lib,
  fetchNpmDeps,
  cargo-tauri,
  nodejs_24,
  npmHooks,
  pkg-config,
  stdenv,
  openssl,
  wrapGAppsHook4,
  webkitgtk_4_1,
}:
rustPlatform.buildRustPackage (finalAttrs: {
  pname = "mcfs";
  version = "0.1.0";
  src = lib.cleanSourceWith {
    src = lib.cleanSource ./.;
    filter = path: _: !builtins.elem (baseNameOf path) ["node_modules" "target" "dist"];
  };

  cargoLock.lockFile = ./src-tauri/Cargo.lock;

  cargoRoot = "src-tauri";
  buildAndTestSubdir = finalAttrs.cargoRoot;

  doCheck = false;

  npmDeps = fetchNpmDeps {
    name = "${finalAttrs.pname}-${finalAttrs.version}-npm-deps";
    inherit (finalAttrs) src;
    hash = "sha256-Vq1IklntsB0TF5NEzeflpuJIskS5KwCOXLGX0+Uk0tw=";
  };

  nativeBuildInputs =
    [
      cargo-tauri.hook
      nodejs_24
      npmHooks.npmConfigHook
      pkg-config
    ]
    ++ lib.optionals stdenv.hostPlatform.isLinux [wrapGAppsHook4];

  buildInputs =
    [
      openssl
    ]
    ++ lib.optionals stdenv.hostPlatform.isLinux [
      webkitgtk_4_1
    ];
})
