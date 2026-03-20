extern crate self as compaq;

pub use compaq_core::{compress::Compress, CompaqError, Result, compress_identity_impl};
pub use compaq_derive::{compress, __SilenceErrors};

#[cfg(test)]
mod tests {
    use std::collections::HashMap;

    use super::{CompaqError, Compress, compress};

    #[compress(CompressedOrderedSubset)]
    #[derive(Clone, Debug, PartialEq)]
    struct OrderedSubset {
        #[order]
        values: HashMap<String, u32>,
    }

    #[compress(CompressedPackedFlags)]
    #[derive(Clone, Debug, PartialEq)]
    struct PackedFlags {
        #[pack]
        alpha: bool,
        #[pack]
        beta: bool,
        gamma: u8,
        #[pack]
        delta: bool,
    }

    #[test]
    fn ordered_policy_can_encode_subset_of_hash_map_keys() {
        let state = OrderedSubset {
            values: HashMap::from([
                ("alpha".to_string(), 1),
                ("beta".to_string(), 2),
                ("gamma".to_string(), 3),
            ]),
        };

        let compressed = match state.deflate(vec!["gamma".to_string(), "alpha".to_string()]) {
            Ok(compressed) => compressed,
            Err(_) => panic!("subset policy should serialize"),
        };

        let inflated = match compressed.inflate(vec!["gamma".to_string(), "alpha".to_string()]) {
            Ok(inflated) => inflated,
            Err(_) => panic!("matching subset policy should inflate"),
        };

        assert_eq!(
            inflated.values,
            HashMap::from([
                ("gamma".to_string(), 3),
                ("alpha".to_string(), 1),
            ])
        );
    }

    #[test]
    fn ordered_policy_still_errors_for_missing_map_keys() {
        let state = OrderedSubset {
            values: HashMap::from([("alpha".to_string(), 1)]),
        };

        let error = match state.deflate(vec!["missing".to_string()]) {
            Ok(_) => panic!("unknown policy key should fail"),
            Err(error) => error,
        };

        assert!(matches!(error, CompaqError::DesynchronizedPolicy));
    }

    #[test]
    fn pack_attribute_packs_bool_fields_into_leading_bitfield() {
        let state = PackedFlags {
            alpha: true,
            beta: false,
            gamma: 7,
            delta: true,
        };

        let compressed = state.compress();
        assert_eq!(compressed.__packed_bools, [0b0000_0101]);
        assert_eq!(compressed.gamma, 7);

        let roundtrip = PackedFlags::decompress(compressed);
        assert_eq!(roundtrip, state);
    }
}
