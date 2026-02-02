mod bootp;
mod tftp;

use std::{io, net::UdpSocket, path::Path, thread};

struct State {
  pub spl: Box<[u8]>,
  pub uboot: Box<[u8]>,
  pub image: Box<[u8]>,
}

pub fn flash(spl_path: &Path, uboot_path: &Path, image_path: &Path) {
}
