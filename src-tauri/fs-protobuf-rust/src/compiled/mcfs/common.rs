// Automatically generated rust module for 'common.proto' file

#![allow(non_snake_case)]
#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(unused_imports)]
#![allow(unknown_lints)]
#![allow(clippy::all)]
#![cfg_attr(rustfmt, rustfmt_skip)]


use quick_protobuf::{BytesReader, Result, MessageRead, MessageWrite};
use super::super::*;

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum LogLevel {
    LOG_LEVEL_DATA = 0,
    LOG_LEVEL_DEBUG = 1,
    LOG_LEVEL_WARNING = 2,
    LOG_LEVEL_ERROR = 3,
}

impl Default for LogLevel {
    fn default() -> Self {
        LogLevel::LOG_LEVEL_DATA
    }
}

impl From<i32> for LogLevel {
    fn from(i: i32) -> Self {
        match i {
            0 => LogLevel::LOG_LEVEL_DATA,
            1 => LogLevel::LOG_LEVEL_DEBUG,
            2 => LogLevel::LOG_LEVEL_WARNING,
            3 => LogLevel::LOG_LEVEL_ERROR,
            _ => Self::default(),
        }
    }
}

impl<'a> From<&'a str> for LogLevel {
    fn from(s: &'a str) -> Self {
        match s {
            "LOG_LEVEL_DATA" => LogLevel::LOG_LEVEL_DATA,
            "LOG_LEVEL_DEBUG" => LogLevel::LOG_LEVEL_DEBUG,
            "LOG_LEVEL_WARNING" => LogLevel::LOG_LEVEL_WARNING,
            "LOG_LEVEL_ERROR" => LogLevel::LOG_LEVEL_ERROR,
            _ => Self::default(),
        }
    }
}

