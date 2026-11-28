//! Unit tests for the functions declared in `servo/src/tool`.
//!
//! Each source file from `src/tool` is pulled into a dedicated module with
//! `include!`. That keeps the test crate scoped to the tool directory while
//! still allowing the private helpers declared there (platform/repository
//! lookups, date parsing, etc.) to be exercised directly.
//!
//! Only functions that can be exercised without context from outside the tool
//! directory are tested. The remaining functions are documented below with the
//! additional context that would be required to test them:
//!
//! - `clean::clean`: needs a writable deployment cache at the OS-specific
//!   location returned by `deploy::locate_cache` (`/var/cache/servo` on Linux,
//!   which needs elevated privileges) plus a disposable `~/.servo` directory.
//! - `deploy::Repository::fetch_latest`: needs the private YJSP GitHub remotes
//!   and credentials, plus either a populated local git cache or network
//!   access to clone it.
//! - `deploy::Repository::bundle`: needs a fully populated repository cache
//!   with a valid `Cargo.toml` and vendored dependencies.
//! - `deploy::locate_cache`: needs control of the OS-specific cache directory;
//!   on Linux it unconditionally creates `/var/cache/servo`.
//! - `deploy::Target::connect`, `check_rust`, `install_rust`, `transfer`,
//!   `install`, and `deploy`: need live hardware reachable via mDNS, a working
//!   SSH session (including password auth), remote shell commands, and the
//!   offline Rust installer download.
//! - `deploy::deploy`: orchestrates all of the network/SSH operations above
//!   and needs `ArgMatches` from the CLI definition in `src/main.rs`.
//! - `emulate::emulate_flight`, `emulate_sam`, and `emulate`: each enters an
//!   infinite loop after binding to fixed TCP/UDP ports (`5025`, `7201`,
//!   `4573`); they need live peers on those ports and an external stop signal.
//! - `export::export`: needs a ground server listening on
//!   `http://localhost:7200/data/export` and a writable output path.
//! - `locate::locate` (hostname-resolution loop): needs an mDNS-capable
//!   network with the YJSP hosts present; without it each `.local` lookup
//!   stalls for the system resolver timeout. Only the isolated subsystem
//!   validation is tested here.
//! - `run::run`: needs a ground server listening on
//!   `http://localhost:7200/operator/run-sequence`.
//! - `serve::serve`: needs `crate::server` and `crate::interface` (outside the
//!   tool directory), a SQLite database, a Tokio runtime, and a terminal.
//! - `serve::infinite_hang`: intentionally never resolves; it can only be
//!   observed as a pending task inside `serve`'s runtime.
//! - `sql::sql`: needs a ground server on `http://localhost:7200/admin/sql`;
//!   the table formatting is inline in the function and cannot be exercised
//!   without a response from that endpoint.
//! - `upload::upload`: needs a sequence file on disk and a ground server on
//!   `http://localhost:7200/operator/sequence`.
//! - `monitor.rs` contains no functions.

mod deploy_src {
    #![allow(dead_code)]

    include!("../src/tool/deploy.rs");

    #[test]
    fn platform_triple_matches_expected_targets() {
        assert_eq!(Platform::AppleSilicon.triple(), "aarch64-apple-darwin");
        assert_eq!(
            Platform::Beaglebone.triple(),
            "armv7-unknown-linux-gnueabihf"
        );
        assert_eq!(Platform::Meerkat.triple(), "x86_64-unknown-linux-gnu");
    }

    #[test]
    fn platform_triples_are_nonempty_and_distinct() {
        let triples = [
            Platform::AppleSilicon.triple(),
            Platform::Beaglebone.triple(),
            Platform::Meerkat.triple(),
        ];

        assert!(triples.iter().all(|triple| !triple.is_empty()));
        for (index, first) in triples.iter().enumerate() {
            for second in triples.iter().skip(index + 1) {
                assert_ne!(first, second);
            }
        }
    }

    #[test]
    fn platform_default_login_matches_expected_devices() {
        assert_eq!(Platform::AppleSilicon.default_login(), ("none", "none"));
        assert_eq!(Platform::Beaglebone.default_login(), ("debian", "temppwd"));
        assert_eq!(Platform::Meerkat.default_login(), ("yjsp", "yjspfullscale"));
    }

    #[test]
    fn platform_default_login_credentials_are_nonempty() {
        for platform in [
            Platform::AppleSilicon,
            Platform::Beaglebone,
            Platform::Meerkat,
        ] {
            let (user, password) = platform.default_login();
            assert!(!user.is_empty());
            assert!(!password.is_empty());
        }
    }

    #[test]
    fn repository_all_contains_every_variant_once() {
        let all = Repository::all();

        assert_eq!(all.len(), 5);
        assert!(all.contains(&Repository::Servo));
        assert!(all.contains(&Repository::Flight));
        assert!(all.contains(&Repository::Ahrs));
        assert!(all.contains(&Repository::Gui));
        assert!(all.contains(&Repository::Sam));

        let mut unique = all.clone();
        unique.dedup();
        assert_eq!(unique.len(), all.len());
    }

    #[test]
    fn repository_all_has_stable_ordering() {
        let all = Repository::all();

        assert_eq!(all.first(), Some(&Repository::Servo));
        assert_eq!(all.last(), Some(&Repository::Sam));
        assert_eq!(all, Repository::all());
    }

    #[test]
    fn repository_remote_maps_every_variant() {
        assert_eq!(
            Repository::Ahrs.remote(),
            "git@github-research.gatech.edu:YJSP/ahrs"
        );
        assert_eq!(
            Repository::Flight.remote(),
            "git@github-research.gatech.edu:YJSP/flight"
        );
        assert_eq!(
            Repository::Gui.remote(),
            "git@github-research.gatech.edu:YJSP/fs-gui"
        );
        assert_eq!(
            Repository::Sam.remote(),
            "git@github-research.gatech.edu:YJSP/fs-sam-software"
        );
        assert_eq!(
            Repository::Servo.remote(),
            "git@github-research.gatech.edu:YJSP/servo"
        );
    }

    #[test]
    fn repository_remote_urls_are_unique_and_ssh_based() {
        let remotes: Vec<&str> = Repository::all()
            .into_iter()
            .map(Repository::remote)
            .collect();

        assert!(remotes
            .iter()
            .all(|remote| remote.starts_with("git@github-research.gatech.edu:YJSP/")));

        let mut unique = remotes.clone();
        unique.sort_unstable();
        unique.dedup();
        assert_eq!(unique.len(), remotes.len());
    }

    #[test]
    fn repository_display_uses_directory_names() {
        assert_eq!(Repository::Ahrs.to_string(), "ahrs");
        assert_eq!(Repository::Flight.to_string(), "flight");
        assert_eq!(Repository::Gui.to_string(), "gui");
        assert_eq!(Repository::Sam.to_string(), "sam");
        assert_eq!(Repository::Servo.to_string(), "servo");
    }

    #[test]
    fn repository_display_matches_part_of_its_remote() {
        for repository in Repository::all() {
            assert!(
                repository.remote().contains(&repository.to_string()),
                "{} is missing from {}",
                repository,
                repository.remote()
            );
        }
    }

    #[test]
    fn target_new_initializes_identity_fields() {
        let target = Target::new("sam-01", Repository::Sam, Platform::Beaglebone);

        assert_eq!(target.hostname, "sam-01");
        assert_eq!(target.repository, Repository::Sam);
        assert_eq!(target.platform, Platform::Beaglebone);
    }

    #[test]
    fn target_new_starts_without_a_session() {
        let target = Target::new("gui-01", Repository::Gui, Platform::Meerkat);

        assert!(target.session.is_none());
    }
}

mod export_src {
    #![allow(dead_code)]

    include!("../src/tool/export.rs");

    use chrono::TimeZone;

    #[test]
    fn edit_date_time_replaces_the_output_datetime() {
        let naive = NaiveDate::from_ymd_opt(2024, 3, 5)
            .unwrap()
            .and_hms_opt(7, 8, 9)
            .unwrap();
        let mut output = Local
            .with_ymd_and_hms(2000, 1, 1, 0, 0, 0)
            .earliest()
            .unwrap();

        assert!(edit_date_time(naive, &mut output, true));
        assert_eq!(output.naive_local(), naive);
    }

    #[test]
    fn edit_date_time_earliest_and_latest_agree_when_unambiguous() {
        let naive = NaiveDate::from_ymd_opt(2024, 1, 15)
            .unwrap()
            .and_hms_opt(12, 0, 0)
            .unwrap();
        let initial = Local
            .with_ymd_and_hms(2000, 1, 1, 0, 0, 0)
            .earliest()
            .unwrap();
        let mut earliest = initial;
        let mut latest = initial;

        assert!(edit_date_time(naive, &mut earliest, true));
        assert!(edit_date_time(naive, &mut latest, false));
        assert_eq!(earliest, latest);
        assert_eq!(earliest.naive_local(), naive);
    }

    #[test]
    fn edit_time_replaces_only_the_time_component() {
        let mut output = Local
            .with_ymd_and_hms(2024, 1, 2, 3, 4, 5)
            .earliest()
            .unwrap();
        let time = NaiveTime::from_hms_opt(12, 30, 45).unwrap();

        assert!(edit_time(time, &mut output, true));
        assert_eq!(
            output.date_naive(),
            NaiveDate::from_ymd_opt(2024, 1, 2).unwrap()
        );
        assert_eq!(output.time(), time);
    }

    #[test]
    fn edit_time_can_set_midnight() {
        let mut output = Local
            .with_ymd_and_hms(2024, 6, 30, 23, 59, 59)
            .earliest()
            .unwrap();
        let midnight = NaiveTime::from_hms_opt(0, 0, 0).unwrap();

        assert!(edit_time(midnight, &mut output, true));
        assert_eq!(output.time(), midnight);
    }

    #[test]
    fn edit_date_replaces_only_the_date_component() {
        let mut output = Local
            .with_ymd_and_hms(2024, 1, 2, 3, 4, 5)
            .earliest()
            .unwrap();
        let date = NaiveDate::from_ymd_opt(2025, 7, 4).unwrap();

        assert!(edit_date(date, &mut output, true));
        assert_eq!(output.date_naive(), date);
        assert_eq!(output.time(), NaiveTime::from_hms_opt(3, 4, 5).unwrap());
    }

    #[test]
    fn edit_date_can_set_leap_day() {
        let mut output = Local
            .with_ymd_and_hms(2023, 2, 28, 23, 59, 59)
            .earliest()
            .unwrap();
        let leap_day = NaiveDate::from_ymd_opt(2024, 2, 29).unwrap();

        assert!(edit_date(leap_day, &mut output, true));
        assert_eq!(output.date_naive(), leap_day);
        assert_eq!(output.time(), NaiveTime::from_hms_opt(23, 59, 59).unwrap());
    }

    #[test]
    fn parse_date_and_time_accepts_supported_formats() {
        let base = Local
            .with_ymd_and_hms(1987, 11, 13, 0, 0, 0)
            .earliest()
            .unwrap();
        let mut output = base;

        assert!(parse_date_and_time("Aug 13 1:00pm", &mut output, true));
        assert_eq!(
            output.naive_local(),
            NaiveDate::from_ymd_opt(1987, 8, 13)
                .unwrap()
                .and_hms_opt(13, 0, 0)
                .unwrap()
        );

        let mut output = base;
        assert!(parse_date_and_time("02/29/1992 14:00", &mut output, true));
        assert_eq!(
            output.naive_local(),
            NaiveDate::from_ymd_opt(1992, 2, 29)
                .unwrap()
                .and_hms_opt(14, 0, 0)
                .unwrap()
        );
    }

    #[test]
    fn parse_date_and_time_rejects_invalid_input_without_mutating_output() {
        let mut output = Local
            .with_ymd_and_hms(1987, 11, 13, 0, 0, 0)
            .earliest()
            .unwrap();
        let original = output;

        assert!(!parse_date_and_time("not a date", &mut output, true));
        assert_eq!(output, original);
    }

    #[test]
    fn parse_start_empty_defaults_to_midnight_today() {
        let start = parse_start("").expect("an empty start string should default to today");

        assert_eq!(start.time(), NaiveTime::from_hms_opt(0, 0, 0).unwrap());
        assert!(
            (Local::now().date_naive() - start.date_naive())
                .num_days()
                .abs()
                <= 1
        );
    }

    #[test]
    fn parse_start_rejects_invalid_datetime() {
        assert!(parse_start("definitely not a date").is_none());
    }

    #[test]
    fn parse_end_empty_defaults_to_the_current_time() {
        let before = Local::now();
        let end = parse_end("").expect("an empty end string should default to now");
        let after = Local::now();

        assert!(end >= before, "parsed end preceded the call");
        assert!(end <= after, "parsed end followed the call");
    }

    #[test]
    fn parse_end_rejects_invalid_datetime() {
        assert!(parse_end("definitely not a date").is_none());
    }
}

mod locate_src {
    #![allow(dead_code)]

    include!("../src/tool/locate.rs");

    fn arg_matches(args: &[&str]) -> clap::ArgMatches {
        clap::Command::new("locate")
            .arg(clap::Arg::new("subsystem").required(false))
            .try_get_matches_from(args)
            .expect("test arguments should be valid")
    }

    #[test]
    fn locate_rejects_an_unknown_subsystem() {
        let args = arg_matches(&["locate", "router"]);

        let error = locate(&args).expect_err("an unknown subsystem must be rejected");
        assert!(error.to_string().contains("router"));
    }

    #[test]
    fn locate_rejects_an_unrecognized_prefix() {
        let args = arg_matches(&["locate", "telemetry"]);

        let error = locate(&args).expect_err("a prefix without a configured matcher must fail");
        assert!(error.to_string().contains("Invalid subsystem"));
    }
}
