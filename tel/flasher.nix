{
  target,
  image,
  coreutils,
  bmaptool,
  writeShellScriptBin,
  stdenv,
}: let
  darwinFlash = ''
    ${coreutils}/bin/dd \
      if="$IMAGE" \
      of="$DEVICE" \
      bs=4M \
      status=progress \
      conv=fsync
  '';

  linuxFlash = ''
    ${bmaptool}/bin/bmaptool copy --nobmap "$IMAGE" "$DEVICE"
  '';
in
  writeShellScriptBin "${target}-flasher" ''
    set -euo pipefail

    DEVICE="''${1:-}"

    if [[ -z "$DEVICE" ]]; then
      echo "error: no device path specified" >&2
      echo "usage: ${target}-flasher <device>" >&2
      exit 1
    fi

    # Resolve exactly one image.
    shopt -s nullglob
    images=("${image}/sd-image/"nixos-*.img)

    if (( ''${#images[@]} != 1 )); then
      echo "error: expected exactly one image, found ''${#images[@]}" >&2
      exit 1
    fi

    IMAGE="''${images[0]}"

    # Validate the destination device.
    ${
      if stdenv.isDarwin
      then ''
        if [[ ! -b "$DEVICE" && ! -c "$DEVICE" ]]; then
          echo "error: not a device: $DEVICE" >&2
          exit 1
        fi
      ''
      else ''
        if [[ ! -b "$DEVICE" ]]; then
          echo "error: not a block device: $DEVICE" >&2
          exit 1
        fi
      ''
    }

    echo "Image:  $IMAGE"
    echo "Device: $DEVICE"
    echo "WARNING: All data on this device will be overwritten."

    ${
      if stdenv.isDarwin
      then darwinFlash
      else linuxFlash
    }

    echo "Flash complete."
  ''
