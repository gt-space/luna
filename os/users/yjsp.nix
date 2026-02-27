{ ... }:
{
  users.users.yjsp = {
    isNormalUser = true;
    password = "";
    extraGroups = [
      "dialout"
      "gpio"
      "i2c"
      "spi"
      "wheel"
    ];
  };
}
