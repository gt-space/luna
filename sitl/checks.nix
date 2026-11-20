{
  servo,
  flight,
  harness,
  common,
  testers,
  iproute2,
  iptables,
  procps,
  python3,
  ...
}: let
  mkVmCheck = scenario:
    testers.runNixOSTest {
      name = "isolab-${scenario}";

      nodes.machine = {
        virtualisation.memorySize = 2048;
        virtualisation.cores = 2;

        networking.useDHCP = false;
        networking.dhcpcd.enable = false;

        environment.systemPackages = [
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
in {
  default-source-umbilical = mkVmCheck "default-source-umbilical";
  radio-survives-disconnect = mkVmCheck "radio-survives-disconnect";
  vespula-radio-forwarding = mkVmCheck "vespula-radio-forwarding";
  radio-without-sam = mkVmCheck "radio-without-sam";
}
