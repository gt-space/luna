{ self, ... }:
{
  apps = pkgs:
    let
      inherit (pkgs.stdenv.hostPlatform) system;
      mkRunApp =
        name:
        check:
        let
          runner = pkgs.writeShellScriptBin "isolab-${name}" ''
            exec ${pkgs.nix}/bin/nix build "${builtins.toString ../.}#checks.${system}.isolab.${check}" -L --rebuild "$@"
          '';
        in
        {
          type = "app";
          program = "${runner}/bin/isolab-${name}";
        };
    in
    {
      isolab = {
        servo-flight-default-source = mkRunApp "servo-flight-default-source" "servo-flight-default-source";
        servo-flight-disconnect = mkRunApp "servo-flight-disconnect" "servo-flight-disconnect";
        servo-flight-vespula = mkRunApp "servo-flight-vespula" "servo-flight-vespula";
        servo-flight-vm = mkRunApp "servo-flight-vm" "servo-flight-vm";
      };
    };

  checks = pkgs:
    let
      inherit (pkgs.stdenv.hostPlatform) system;
      servo = self.packages.${system}.servo.servo;
      flight = self.packages.${system}.flight2.flight-computer;
      harness = self.packages.${system}.sitl.isolab;
      common = self.packages.${system}.common.sequences;
      mkVmCheck =
        scenario:
        pkgs.testers.runNixOSTest {
          name = "isolab-${scenario}";

          nodes.machine = { pkgs, ... }: {
            virtualisation.memorySize = 2048;
            virtualisation.cores = 2;

            networking.useDHCP = false;
            networking.dhcpcd.enable = false;

            environment.systemPackages = with pkgs; [
              iproute2
              iptables
              procps
              python3
            ];
          };

          testScript = ''
            import time

            start = time.monotonic()
            start_all()
            machine.wait_for_unit("multi-user.target")
            boot_seconds = time.monotonic() - start

            print(f"isolab VM booted in {boot_seconds:.2f}s")

            status, output = machine.execute(
              "${harness}/bin/isolab "
              "--servo-bin ${servo}/bin/servo "
              "--flight-bin ${flight}/bin/flight-computer "
              "--common-lib ${common}/lib/libcommon.so "
              "--workdir /tmp/isolab-${scenario} "
              "--scenario ${scenario} "
              "2>&1"
            )
            print(output)
            if status != 0:
              raise Exception(
                f"isolab scenario ${scenario} failed with exit code {status}\n{output}"
              )
          '';
        };
    in
    {
      isolab = {
        servo-flight-default-source = mkVmCheck "default-source-umbilical";
        servo-flight-disconnect = mkVmCheck "radio-survives-disconnect";
        servo-flight-vespula = mkVmCheck "vespula-radio-forwarding";
        servo-flight-vm = mkVmCheck "radio-survives-disconnect";
      };
    };

  packages = pkgs:
    let
      workspaceSrc = builtins.path {
        path = ../.;
        name = "luna-source";
      };

      cargoLock = {
        lockFile = ../Cargo.lock;
        outputHashes = {
          "hdf5-0.8.1" = "sha256-3tHQeGu/6Rn2aicoVHZG6lXkx9XNktka/x/zsOawypc=";
          "hdf5-derive-0.8.1" = "sha256-3tHQeGu/6Rn2aicoVHZG6lXkx9XNktka/x/zsOawypc=";
          "hdf5-src-0.8.1" = "sha256-3tHQeGu/6Rn2aicoVHZG6lXkx9XNktka/x/zsOawypc=";
          "hdf5-sys-0.8.1" = "sha256-3tHQeGu/6Rn2aicoVHZG6lXkx9XNktka/x/zsOawypc=";
          "hdf5-types-0.8.1" = "sha256-3tHQeGu/6Rn2aicoVHZG6lXkx9XNktka/x/zsOawypc=";
        };
      };

      isolab = pkgs.rustPlatform.buildRustPackage {
        pname = "isolab";
        version = "0.1.0";
        src = workspaceSrc;
        inherit cargoLock;
        doCheck = false;
        cargoBuildFlags = [
          "--package" "isolab"
          "--bin" "isolab"
        ];
        nativeBuildInputs = with pkgs; [
          pkg-config
        ];
        buildInputs = with pkgs; [
          openssl
        ];
      };
    in
    {
      sitl = {
        inherit isolab;
      };
    };
}
