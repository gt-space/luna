//! Closed loop at the flight software's rate: the harness's scheduled LQR
//! (the same law as LQR_Gain_Scheduler, u = unom - K1 (x - xref) - K2)
//! flying the plant at FSW_PERIOD, for checking a vehicle's gains hold up
//! at 100 Hz before flight2 flies them.
//!
//!   cargo run --release --example ctv_lqr -- <gains dir> [out.csv]
//!
//! <gains dir> is the hopper repo's fsw_bridge/export_plant_gains output
//! (tgrid, xref, k1flat, k2grid, unom .csv). Prints apex, touchdown time and
//! landing offset; out.csv gets t, x[13], u[4] per period.

use std::{env, fs, io::Write, path::Path};

use hopper_plant::{Plant, FSW_PERIOD};

fn table(dir: &Path, name: &str, cols: usize) -> Vec<Vec<f64>> {
    let path = dir.join(format!("{name}.csv"));
    let t: Vec<Vec<f64>> = fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("{}: {e}", path.display()))
        .lines()
        .filter(|l| !l.trim().is_empty())
        .map(|l| l.split(',').map(|v| v.trim().parse().unwrap()).collect())
        .collect();
    assert!(
        t.iter().all(|r| r.len() == cols),
        "{name}.csv is not {cols} wide"
    );
    t
}

struct Lqr {
    tgrid: Vec<f64>,
    xref: Vec<Vec<f64>>,
    k1: Vec<Vec<f64>>,
    k2: Vec<Vec<f64>>,
    unom: Vec<Vec<f64>>,
}

impl Lqr {
    fn command(&self, t: f64, x: &[f64; 13]) -> [f64; 4] {
        let n = self.tgrid.len();
        let (i, w) = if t <= self.tgrid[0] {
            (0, 0.0)
        } else if t >= self.tgrid[n - 1] {
            (n - 2, 1.0)
        } else {
            let i = (0..n - 1)
                .find(|&k| self.tgrid[k] <= t && t < self.tgrid[k + 1])
                .unwrap();
            (i, (t - self.tgrid[i]) / (self.tgrid[i + 1] - self.tgrid[i]))
        };
        let lerp = |a: &[Vec<f64>], j: usize| (1.0 - w) * a[i][j] + w * a[i + 1][j];
        let xref: Vec<f64> = (0..13).map(|j| lerp(&self.xref, j)).collect();
        let mut q: Vec<f64> = x[9..13].to_vec();
        if (0..4).map(|j| q[j] * xref[9 + j]).sum::<f64>() < 0.0 {
            q.iter_mut().for_each(|v| *v = -*v);
        }
        let dx: Vec<f64> = (0..13)
            .map(|j| if j < 9 { x[j] } else { q[j - 9] } - xref[j])
            .collect();
        std::array::from_fn(|r| {
            // K1flat is row-major: K1(r, c) = k1flat[13 r + c], as MATLAB's
            // reshape(Kflat, 13, 4)' reads it
            let k1dx: f64 = (0..13).map(|c| lerp(&self.k1, 13 * r + c) * dx[c]).sum();
            lerp(&self.unom, r) - k1dx - lerp(&self.k2, r)
        })
    }
}

fn main() {
    let args: Vec<String> = env::args().collect();
    let dir = Path::new(args.get(1).expect("usage: ctv_lqr <gains dir> [out.csv]"));
    let lqr = Lqr {
        tgrid: table(dir, "tgrid", 1).into_iter().map(|r| r[0]).collect(),
        xref: table(dir, "xref", 13),
        k1: table(dir, "k1flat", 52),
        k2: table(dir, "k2grid", 4),
        unom: table(dir, "unom", 4),
    };
    let mut out = args.get(2).map(|p| fs::File::create(p).unwrap());
    let mut plant = Plant::new().unwrap();
    let t_end = lqr.tgrid[lqr.tgrid.len() - 1] + 5.0;
    let (mut apex, mut touchdown) = (0.0f64, None);
    let mut last = [0.0; 13];
    while plant.time() < t_end {
        let x = plant.state().unwrap();
        let u = lqr.command(plant.time(), &x);
        if let Some(f) = out.as_mut() {
            let row: Vec<String> = std::iter::once(plant.time())
                .chain(x)
                .chain(u)
                .map(|v| format!("{v:.9}"))
                .collect();
            writeln!(f, "{}", row.join(",")).unwrap();
        }
        apex = apex.max(-x[2]);
        last = x;
        let p = plant.run_period(&u).unwrap();
        if p.touchdown {
            touchdown = Some(p.time);
            break;
        }
    }
    println!(
        "closed loop at {:.0} Hz: apex {apex:.3} m",
        1.0 / FSW_PERIOD
    );
    match touchdown {
        Some(t) => println!("touchdown at {t:.2} s"),
        None => println!("no touchdown by {t_end:.1} s"),
    }
    println!(
        "last period: pos NED [{:.3}, {:.3}, {:.3}] m, vel NED [{:.3}, {:.3}, {:.3}] m/s, q [{:.4}, {:.4}, {:.4}, {:.4}]",
        last[0], last[1], last[2], last[3], last[4], last[5], last[9], last[10], last[11], last[12]
    );
}
