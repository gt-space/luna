use proc_macro2::{Span, TokenStream};
use quote::{quote, quote_spanned};
use syn::{AttrStyle, punctuated::Punctuated, Attribute, Data, DeriveInput, Error, Field, Fields, GenericArgument, Ident, Meta, Path, PathArguments, Token, Type, TypePath, parse_macro_input, parse_quote, spanned::Spanned};

// Attributes:
// Exclude ("exclude"): Removes a field from compression. 
const EXCLUDE_ATTRIBUTE_STRING: &str = "exclude";

// Freeze ("freeze"): Prevents a field from being compressed.
const FREEZE_ATTRIBUTE_STRING: &str = "freeze";

// Order ("order"): 
const ORDER_ATTRIBUTE_STRING: &str = "order";
const PACK_ATTRIBUTE_STRING: &str = "pack";

#[derive(Clone)]
enum Tag<'a> {
    Excluded,
    Frozen,
    Ordered { is_frozen: bool, k: &'a Type, v: &'a Type },
    Packed,
}

#[derive(Clone)]
struct AttributedField<'a> {
    field: &'a Field,
    tag: Option<Tag<'a>>,
}

impl<'a> AttributedField<'a> {
    fn new(field: &'a Field, is_excluded: bool, is_frozen: bool, is_ordered: bool, is_packed: bool) -> Self {
        if is_excluded {
            AttributedField { field, tag: Some(Tag::Excluded) }
        } else if is_ordered {
            let Type::Path(path) = &field.ty else {
                panic!("#[order] attribute must be attributed to a type using a type path.");
            };

            let segment = path.path.segments.last().expect("An empty type path cannot use the #[order] attribute.");
            let PathArguments::AngleBracketed(args) = &segment.arguments else {
                panic!("A HashMap attributed with the #[order] attribute must have a angle-bracketed generic argument list.");
            };

            let mut iter = args.args.iter();
            let (Some(GenericArgument::Type(k)), Some(GenericArgument::Type(v))) = (iter.next(), iter.next()) else {
                panic!("The Hashmap type must have at least two generic arguments to use the #[order] attribute.");
            };
            
            AttributedField { field, tag: Some(Tag::Ordered { is_frozen, k, v }) }
        } else if is_packed {
            AttributedField { field, tag: Some(Tag::Packed) }
        } else if is_frozen {
            AttributedField { field, tag: Some(Tag::Frozen) }
        } else {
            AttributedField { field, tag: None }
        }
    }
}

fn process_field_attributes<'a>(raw_fields: impl Iterator<Item = &'a Field>) -> syn::Result<Vec<AttributedField<'a>>> {
    fn is_bool_type(ty: &Type) -> bool {
        matches!(
            ty,
            Type::Path(TypePath { path, .. })
                if path.segments.last().is_some_and(|segment| segment.ident == "bool")
        )
    }

    fn detect_attribution_rule_violations(field: &Field, excluded: Option<Span>, frozen: Option<Span>, ordered: Option<Span>, packed: Option<Span>) -> Option<Error> {
        let mut accumulated_error: Option<Error> = None;

        if let Some(excluded) = excluded && let Some(frozen) = frozen {
            let mut error = syn::Error::new(excluded, "`#[exclude]` and `#[freeze]` cannot be attributed to a field simultaneously.");
            error.combine(syn::Error::new(frozen, "`#[freeze]` and `#[exclude]` cannot be attributed to a field simultaneously."));
            
            if let Some(ref mut e) = accumulated_error {
                e.combine(error);
            } else {
                accumulated_error = Some(error);
            }
        }

        if let Some(excluded) = excluded && let Some(ordered) = ordered {
            let mut error = syn::Error::new(excluded, "`#[exclude]` and `#[order]` cannot be attributed to a field simultaneously.");
            error.combine(syn::Error::new(ordered, "`#[order]` and `#[exclude]` cannot be attributed to a field simultaneously."));
            
            if let Some(ref mut e) = accumulated_error {
                e.combine(error);
            } else {
                accumulated_error = Some(error);
            }
        }

        if let Some(excluded) = excluded && let Some(packed) = packed {
            let mut error = syn::Error::new(excluded, "`#[exclude]` and `#[pack]` cannot be attributed to a field simultaneously.");
            error.combine(syn::Error::new(packed, "`#[pack]` and `#[exclude]` cannot be attributed to a field simultaneously."));

            if let Some(ref mut e) = accumulated_error {
                e.combine(error);
            } else {
                accumulated_error = Some(error);
            }
        }

        if let Some(frozen) = frozen && let Some(packed) = packed {
            let mut error = syn::Error::new(frozen, "`#[freeze]` and `#[pack]` cannot be attributed to a field simultaneously.");
            error.combine(syn::Error::new(packed, "`#[pack]` and `#[freeze]` cannot be attributed to a field simultaneously."));

            if let Some(ref mut e) = accumulated_error {
                e.combine(error);
            } else {
                accumulated_error = Some(error);
            }
        }

        if let Some(ordered) = ordered && let Some(packed) = packed {
            let mut error = syn::Error::new(ordered, "`#[order]` and `#[pack]` cannot be attributed to a field simultaneously.");
            error.combine(syn::Error::new(packed, "`#[pack]` and `#[order]` cannot be attributed to a field simultaneously."));

            if let Some(ref mut e) = accumulated_error {
                e.combine(error);
            } else {
                accumulated_error = Some(error);
            }
        }

        if ordered.is_some() {
            // Checks if the type of a field with the #[order] attribute is a HashMap.
            if let Type::Path(TypePath { path, .. }) = &field.ty
                && let Some(segment) = path.segments.last() 
                && segment.ident == "HashMap"
                && let PathArguments::AngleBracketed(args) = &segment.arguments
                && args.args.iter().len() >= 2
            {} else {
                let error = syn::Error::new(field.span(), "Field attributed with `#[order]` attribute must be of type `HashMap<K, V>`.");
                if let Some(ref mut e) = accumulated_error {
                    e.combine(error);
                } else {
                    accumulated_error = Some(error);
                }
            }
        }

        if let Some(packed_span) = packed && !is_bool_type(&field.ty) {
            let error = syn::Error::new(packed_span, "Field attributed with `#[pack]` must be of type `bool`.");
            if let Some(ref mut e) = accumulated_error {
                e.combine(error);
            } else {
                accumulated_error = Some(error);
            }
        }

        accumulated_error
    }
    
    let mut formatted_fields = Vec::new();

    for field in raw_fields {
        let mut excluded = None;
        let mut frozen = None;
        let mut ordered = None;
        let mut packed = None;

        // attach attribute to some field
        for attribute in &field.attrs {
            if !matches!(attribute.style, AttrStyle::Outer) {
                continue;
            }

            let Meta::Path(ref p) = attribute.meta else {
                continue;
            };

            if p.is_ident(EXCLUDE_ATTRIBUTE_STRING) && excluded.is_none() {
                excluded = Some(attribute.span());
            }

            if p.is_ident(FREEZE_ATTRIBUTE_STRING) && frozen.is_none() {
                frozen = Some(attribute.span());
            }

            if p.is_ident(ORDER_ATTRIBUTE_STRING) && ordered.is_none() {
                ordered = Some(attribute.span());
            }

            if p.is_ident(PACK_ATTRIBUTE_STRING) && packed.is_none() {
                packed = Some(attribute.span());
            }
        }

        if let Some(error) = detect_attribution_rule_violations(field, excluded, frozen, ordered, packed) {
            return Err(error);
        }

        formatted_fields.push(AttributedField::new(field, excluded.is_some(), frozen.is_some(), ordered.is_some(), packed.is_some()));
    }

    Ok(formatted_fields)
}

fn packed_bool_fields<'a>(fields: &'a Vec<AttributedField<'a>>) -> Vec<&'a AttributedField<'a>> {
    fields.iter().filter(|f| matches!(f.tag, Some(Tag::Packed))).collect()
}

fn get_compressed_struct_name(name: &Ident, has_ordered_member: bool) -> Ident {
    if has_ordered_member {
        Ident::new(&format!("Unordered{}", name), name.span())
    } else {
        name.clone()
    }
}

fn generate_struct_members<'a>(fields: &'a Vec<AttributedField<'a>>, enforce_ordering: bool) -> impl Iterator<Item = TokenStream> {    
    fields.iter().filter_map(move |f| {
        let name = f.field.ident.as_ref().unwrap();
        let ty = &f.field.ty;
        
        match f.tag {
            Some(Tag::Excluded) => None,
            Some(Tag::Packed) => None,
            Some(Tag::Frozen) => Some(quote_spanned! {f.field.span()=> #name: #ty, }),
            Some(Tag::Ordered { is_frozen, k, v }) => {
                let inner_type: Type = if is_frozen { parse_quote! { #v } } else { parse_quote! { <#v as ::compaq::Compress>::Compressed } };
                
                if enforce_ordering {
                    Some(quote_spanned! {f.field.span()=> #name: ::std::vec::Vec<#inner_type>, })
                } else {
                    Some(quote_spanned! {f.field.span()=> #name: ::std::collections::HashMap<#k, #inner_type>, })
                }
            }
            None => Some(quote_spanned! {f.field.span()=> #name: <#ty as ::compaq::Compress>::Compressed, }),
        }
    })
}

fn generate_compressed_struct(input: &DeriveInput, compressed_name: &Ident, fields: &Vec<AttributedField>, has_ordered_member: bool) -> TokenStream {
    let packed_bool_fields = packed_bool_fields(fields);
    let transformed_fields = generate_struct_members(fields, false);

    let name = get_compressed_struct_name(compressed_name, has_ordered_member);
    let attributes = strip_compress_attribute(&input.attrs);
    let vis = &input.vis;
    let packed_len = packed_bool_fields.len().div_ceil(8);
    let packed_field = (packed_len > 0).then(|| quote! { __packed_bools: [u8; #packed_len], });

    quote_spanned! {input.span()=>
        #[allow(dead_code)]
        #(#attributes)*
        #vis struct #name {
            #packed_field
            #(#transformed_fields)*
        }
    }
}

fn generate_trait_assertions(fields: &Vec<AttributedField>) -> TokenStream {
    fn assert_impl(ty: &Type, path: Path) -> TokenStream {
        quote_spanned! {ty.span()=> 
            const _: () = {
                const fn assert_impl<T: #path>() {}
                assert_impl::<#ty>();
            };
        }
    }
    
    // use flat_map()
    let mut asserts = Vec::new();
    for field in fields {
        let ty = &field.field.ty;

        match field.tag {
            Some(Tag::Excluded) => asserts.push(assert_impl(ty, parse_quote! { ::core::default::Default })),
            Some(Tag::Packed) => {},
            Some(Tag::Frozen) => asserts.push(assert_impl(ty, parse_quote! { ::core::clone::Clone })),
            Some(Tag::Ordered { is_frozen, k: _, v }) if is_frozen => asserts.push(assert_impl(v, parse_quote! { ::core::clone::Clone })),
            // The required traits for these fields are Compress, and we get that type check for free with the generated `<#ty as Compress>` statements. 
            Some(Tag::Ordered { .. }) | None => {},
        }
        
        if let Some(Tag::Ordered { k, .. }) = field.tag {
            asserts.push(assert_impl(k, parse_quote! { ::core::hash::Hash }));
            asserts.push(assert_impl(k, parse_quote! { ::core::cmp::Eq }));
        }
    }
    
    quote! {
        #(#asserts)*
    }
}


fn generate_ordered_struct(input: &DeriveInput, compressed_name: &Ident, fields: &Vec<AttributedField>) -> TokenStream {
    let packed_bool_fields = packed_bool_fields(fields);
    let transformed_fields = generate_struct_members(fields, true);

    let name = get_compressed_struct_name(compressed_name, false);
    let attributes = strip_compress_attribute(&input.attrs);
    let vis = &input.vis;
    let packed_len = packed_bool_fields.len().div_ceil(8);
    let packed_field = (packed_len > 0).then(|| quote! { __packed_bools: [u8; #packed_len], });

    quote_spanned! {input.span()=>
        #[allow(dead_code)]
        #(#attributes)*
        #vis struct #name {
            #packed_field
            #(#transformed_fields)*
        }
    }
}

fn generate_compress_impl(input: &DeriveInput, compressed_name: &Ident, fields: &Vec<AttributedField>, is_ordered: bool) -> TokenStream {
    let compress_name = get_compressed_struct_name(compressed_name, is_ordered);
    let name = &input.ident;
    let packed_bool_fields = packed_bool_fields(fields);
    let packed_len = packed_bool_fields.len().div_ceil(8);
    let packed_compress = if packed_len > 0 {
        let setters = packed_bool_fields.iter().enumerate().map(|(index, field)| {
            let name = field.field.ident.as_ref().unwrap();
            let byte_index = index / 8;
            let bit_index = index % 8;
            quote_spanned! {field.field.span()=>
                if self.#name {
                    __packed_bools[#byte_index] |= 1 << #bit_index;
                }
            }
        });
        Some(quote! {
            let mut __packed_bools = [0u8; #packed_len];
            #(#setters)*
        })
    } else {
        None
    };
    let packed_compress_initializer = (packed_len > 0).then(|| quote! { __packed_bools, });

    let compress_initializers = fields.iter().filter_map(|f| {
        let name = f.field.ident.as_ref().unwrap();
        let ty = &f.field.ty;
        
        match f.tag {
            Some(Tag::Excluded) => None,
            Some(Tag::Packed) => None,
            Some(Tag::Frozen) => Some(quote_spanned! {f.field.span()=> #name: ::core::clone::Clone::clone(&self.#name), }),
            Some(Tag::Ordered { is_frozen, .. }) if is_frozen => Some(quote_spanned! {f.field.span()=> #name: ::core::clone::Clone::clone(&self.#name), }),
            Some(Tag::Ordered { .. }) | None => Some(quote_spanned! {f.field.span()=> #name: <#ty as ::compaq::Compress>::compress(&self.#name), })
        }
    });

    let packed_decompress = packed_bool_fields.iter().enumerate().map(|(index, field)| {
        let name = field.field.ident.as_ref().unwrap();
        let byte_index = index / 8;
        let bit_index = index % 8;
        quote_spanned! {field.field.span()=> #name: val.__packed_bools[#byte_index] & (1 << #bit_index) != 0, }
    });

    let decompress_initializers = fields.iter().filter_map(|f| {
        let name = f.field.ident.as_ref().unwrap();
        let ty = &f.field.ty;
        
        match f.tag {
            Some(Tag::Packed) => None,
            Some(Tag::Excluded) => Some(quote_spanned! {f.field.span()=> #name: ::core::default::Default::default(), }),
            Some(Tag::Frozen) => Some(quote_spanned! {f.field.span()=> #name: val.#name, }),
            Some(Tag::Ordered { is_frozen, .. }) if is_frozen => Some(quote_spanned! {f.field.span()=> #name: val.#name, }),
            Some(Tag::Ordered { .. }) | None => Some(quote_spanned! {f.field.span()=> #name: <#ty as ::compaq::Compress>::decompress(val.#name), })
        }
    });

    quote! {
        #[automatically_derived]
        impl ::compaq::Compress for #name {
            type Compressed = #compress_name;

            fn compress(&self) -> Self::Compressed {
                #packed_compress
                Self::Compressed {
                    #packed_compress_initializer
                    #(#compress_initializers)*
                }
            }

            fn decompress(val: Self::Compressed) -> Self {
                Self {
                    #(#packed_decompress)*
                    #(#decompress_initializers)*
                }
            }
        }
    }
}

fn isolate_ordered_fields<'a>(fields: &'a Vec<AttributedField<'a>>) -> impl Iterator<Item = &'a AttributedField<'a>> {
    fields.iter().filter(move |f| matches!(f.tag, Some(Tag::Ordered { .. })))
}

fn generate_methods(input: &DeriveInput, compressed_name: &Ident, fields: &Vec<AttributedField>) -> TokenStream {
    let name = &input.ident;
    let vis = &input.vis;
    let compressed_name = get_compressed_struct_name(compressed_name, false);
    let packed_bool_fields = packed_bool_fields(fields);
    let packed_len = packed_bool_fields.len().div_ceil(8);

    let ordered_fields: Vec<&AttributedField<'_>> = isolate_ordered_fields(fields).collect();
    let deflate_policy_parameters = ordered_fields.iter().map(|f| {
        let Some(Tag::Ordered { k, .. }) = f.tag else {
            panic!("Failed to generate policy parameters for `inflate()`.");
        };
        let name = Ident::new(&format!("{}_policy", f.field.ident.as_ref().unwrap()), f.field.span());
        
        quote_spanned! {f.field.span()=>
            , #name: ::std::vec::Vec<#k>
        }
    });
    let inflate_policy_parameters = deflate_policy_parameters.clone();

    // generates the logic to convert HashMap<K, V> + Vec<K> -> Vec<V>
    let deflate_ordered_logic = ordered_fields.iter().map(|f| {
        let name = f.field.ident.as_ref().unwrap();
        let policy_name = Ident::new(&format!("{}_policy", f.field.ident.as_ref().unwrap()), f.field.span());
        let Some(Tag::Ordered { is_frozen, k: _, v }) = f.tag else {
            panic!("Failed to generate deflation logic for ordered parameters.");
        };

        let mut v = v.clone();
        let mut op = quote! { cloned() };
        if !is_frozen {
            op = quote! { map(|v| <#v as ::compaq::Compress>::compress(v)) };
            v = parse_quote! { <#v as ::compaq::Compress>::Compressed };
        }

        quote_spanned! {f.field.span()=>
            let #name = #policy_name.iter().map(|k| self.#name.get(k).#op.ok_or(::compaq::CompaqError::DesynchronizedPolicy)).collect::<::compaq::Result<::std::vec::Vec<#v>>>()?;
        }
    });

    let inflate_ordered_logic = ordered_fields.iter().map(|f| {
        let name = f.field.ident.as_ref().unwrap();
        let policy_name = Ident::new(&format!("{}_policy", f.field.ident.as_ref().unwrap()), f.field.span());
        let Some(Tag::Ordered { is_frozen, k, v }) = f.tag else {
            panic!("Failed to generate inflation logic for ordered parameters.");
        };

        let mut op: TokenStream = TokenStream::new();
        if !is_frozen {
            op = quote! { .map(<#v as ::compaq::Compress>::decompress) };
        }

        quote_spanned! {f.field.span()=>
            if self.#name.len() != #policy_name.len() {
                return ::core::result::Result::Err(::compaq::CompaqError::DesynchronizedPolicy);
            }

            let #name = #policy_name.iter().cloned().zip(self.#name.into_iter()#op).collect::<::std::collections::HashMap<#k, #v>>();
        }
    });

    let deflate_initializers = fields.iter().filter_map(|f| {
        let name = f.field.ident.as_ref().unwrap();
        let ty = &f.field.ty;
        
        match f.tag {
            Some(Tag::Excluded) => None,
            Some(Tag::Packed) => None,
            Some(Tag::Frozen) => Some(quote_spanned! {f.field.span()=> #name: ::core::clone::Clone::clone(&self.#name), }),
            Some(Tag::Ordered { .. }) => Some(quote_spanned! {f.field.span()=> #name, }),
            None => Some(quote_spanned! {f.field.span()=> #name: <#ty as ::compaq::Compress>::compress(&self.#name), }),
        }
    });

    let packed_deflate = if packed_len > 0 {
        let setters = packed_bool_fields.iter().enumerate().map(|(index, field)| {
            let name = field.field.ident.as_ref().unwrap();
            let byte_index = index / 8;
            let bit_index = index % 8;
            quote_spanned! {field.field.span()=>
                if self.#name {
                    __packed_bools[#byte_index] |= 1 << #bit_index;
                }
            }
        });
        Some(quote! {
            let mut __packed_bools = [0u8; #packed_len];
            #(#setters)*
        })
    } else {
        None
    };
    let packed_deflate_initializer = (packed_len > 0).then(|| quote! { __packed_bools, });

    let packed_inflate = packed_bool_fields.iter().enumerate().map(|(index, field)| {
        let name = field.field.ident.as_ref().unwrap();
        let byte_index = index / 8;
        let bit_index = index % 8;
        quote_spanned! {field.field.span()=> #name: self.__packed_bools[#byte_index] & (1 << #bit_index) != 0, }
    });

    let inflate_initializers = fields.iter().filter_map(|f| {
        let name = f.field.ident.as_ref().unwrap();
        let ty = &f.field.ty;
        
        match f.tag {
            Some(Tag::Packed) => None,
            Some(Tag::Excluded) => Some(quote_spanned! { f.field.span()=> #name: ::core::default::Default::default(), }),
            Some(Tag::Frozen) => Some(quote_spanned! {f.field.span()=> #name: self.#name, }),
            Some(Tag::Ordered { .. }) => Some(quote_spanned! {f.field.span()=> #name, }),
            None => Some(quote_spanned! {f.field.span()=> #name: <#ty as ::compaq::Compress>::decompress(self.#name), }),
        }
    });
    
    quote! {
        #[automatically_derived]
        impl #name {
            /// Compresses the object into a smaller format.
            #vis fn deflate(&self #(#deflate_policy_parameters)*) -> ::compaq::Result<#compressed_name> {
                #(#deflate_ordered_logic)*
                #packed_deflate

                ::core::result::Result::Ok(#compressed_name {
                    #packed_deflate_initializer
                    #(#deflate_initializers)*
                })
            }
        }

        #[automatically_derived]
        impl #compressed_name {
            /// Decompresses the object into a more accurate format.
            #vis fn inflate(self #(#inflate_policy_parameters)*) -> ::compaq::Result<#name> {
                #(#inflate_ordered_logic)*

                ::core::result::Result::Ok(#name {
                    #(#packed_inflate)*
                    #(#inflate_initializers)*
                })
            }
        }
    }
}

#[proc_macro_derive(__SilenceErrors, attributes(exclude, freeze, order, pack))]
pub fn derive(_input: proc_macro::TokenStream) -> proc_macro::TokenStream {
    TokenStream::new().into()
}

fn strip_compress_attribute(attrs: &[Attribute]) -> Vec<Attribute> {
    let mut attrs = attrs.to_owned();
    let Some(index) = attrs.iter().position(|val| val.path().is_ident("derive")) else {
        return attrs;
    };
    let attr = attrs.remove(index);

    let args: Punctuated<Path, Token![,]> = attr.parse_args_with(Punctuated::<Path, Token![,]>::parse_terminated).unwrap();
    let filtered: Vec<Path> = args.into_iter().filter(|p| !p.is_ident("Compress")).collect();

    attrs.insert(index, parse_quote! { #[derive(#(#filtered),*)] });
    attrs
}

#[proc_macro_attribute]
pub fn compress(attr: proc_macro::TokenStream, input: proc_macro::TokenStream) -> proc_macro::TokenStream {
    let compressed_name = parse_macro_input!(attr as Ident);
    let input = parse_macro_input!(input as DeriveInput);

    let Data::Struct(structure) = &input.data else {
        panic!("Compress can only work for struct types.");
    };

    let Fields::Named(fields) = &structure.fields else {
        panic!("Compress can only work for named struct types.");
    };

    let fields = match process_field_attributes(fields.named.iter()) {
        Ok(members) => members,
        Err(e) => return e.to_compile_error().into(),
    };

    // checks if any of the struct's fields has the `#[order]` attribute
    let has_ordered_member = fields.iter().any(|f| f.tag.as_ref().is_some_and(|t| matches!(t, Tag::Ordered { .. })));

    // Additional:
    // TODO: Convert all Vecs to Iter for quote generation.
    // TODO: Enforce one instance of each attribute type per field
    // TODO: Convert policy vec to iterator 
    let trait_assertions = generate_trait_assertions(&fields);
    let compressed_struct = generate_compressed_struct(&input, &compressed_name, &fields, has_ordered_member);
    let compress_impl = generate_compress_impl(&input, &compressed_name, &fields, has_ordered_member);
    let methods = generate_methods(&input, &compressed_name, &fields);
    let ordered_struct = if has_ordered_member { generate_ordered_struct(&input, &compressed_name, &fields) } else { TokenStream::new() };

    let generated = quote! {
        #[derive(::compaq::__SilenceErrors)]
        #input
        #ordered_struct
        #compressed_struct
        #compress_impl
        #methods
        #trait_assertions
    };

    generated.into()
}
