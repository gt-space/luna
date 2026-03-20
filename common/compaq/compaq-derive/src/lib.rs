use proc_macro2::{Span, TokenStream};
use quote::{quote, quote_spanned};
use syn::{AttrStyle, punctuated::Punctuated, Attribute, Data, DeriveInput, Error, Field, Fields, GenericArgument, Ident, Meta, Path, PathArguments, Token, Type, TypePath, parse_macro_input, parse_quote, spanned::Spanned};

const MACRO_STRING: &str = "Compress";

// Attributes:
// Exclude ("exclude"): Removes a field from compression. 
const EXCLUDE_ATTRIBUTE_STRING: &str = "exclude";

// Freeze ("freeze"): Prevents a field from being compressed.
const FREEZE_ATTRIBUTE_STRING: &str = "freeze";

// Order ("order"): 
const ORDER_ATTRIBUTE_STRING: &str = "order";

#[derive(Clone)]
enum Tag<'a> {
    Excluded,
    Frozen,
    Ordered { is_frozen: bool, k: &'a Type, v: &'a Type },
}

#[derive(Clone)]
struct AttributedField<'a> {
    field: &'a Field,
    tag: Option<Tag<'a>>,
}

impl<'a> AttributedField<'a> {
    fn new(field: &'a Field, is_excluded: bool, is_frozen: bool, is_ordered: bool) -> Self {
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
        } else if is_frozen {
            AttributedField { field, tag: Some(Tag::Frozen) }
        } else {
            AttributedField { field, tag: None }
        }
    }
}

fn process_field_attributes<'a>(raw_fields: impl Iterator<Item = &'a Field>) -> syn::Result<Vec<AttributedField<'a>>> {
    fn detect_attribution_rule_violations(field: &Field, excluded: Option<Span>, frozen: Option<Span>, ordered: Option<Span>) -> Option<Error> {
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

        accumulated_error
    }
    
    let mut formatted_fields = Vec::new();

    for field in raw_fields {
        let mut excluded = None;
        let mut frozen = None;
        let mut ordered = None;

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
        }

        if let Some(error) = detect_attribution_rule_violations(field, excluded, frozen, ordered) {
            return Err(error);
        }

        formatted_fields.push(AttributedField::new(field, excluded.is_some(), frozen.is_some(), ordered.is_some()));
    }

    Ok(formatted_fields)
}

fn get_compressed_struct_name(input: &DeriveInput, has_ordered_member: bool) -> Ident {
    if has_ordered_member {
        Ident::new(&format!("UnorderedCompressed{}", input.ident), input.ident.span())
    } else {
        Ident::new(&format!("Compressed{}", input.ident), input.ident.span())
    }
}

fn generate_struct_members<'a>(fields: &'a Vec<AttributedField<'a>>, enforce_ordering: bool) -> impl Iterator<Item = TokenStream> {    
    fields.iter().filter_map(move |f| {
        let name = f.field.ident.as_ref().unwrap();
        let ty = &f.field.ty;
        
        match f.tag {
            Some(Tag::Excluded) => None,
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

fn generate_compressed_struct(input: &DeriveInput, fields: &Vec<AttributedField>, has_ordered_member: bool) -> TokenStream {
    let transformed_fields = generate_struct_members(fields, false);

    let name = get_compressed_struct_name(input, has_ordered_member);
    let attributes = strip_compress_attribute(&input.attrs);
    let vis = &input.vis;

    quote_spanned! {input.span()=>
        #[allow(dead_code)]
        #(#attributes)*
        #vis struct #name {
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


fn generate_ordered_struct(input: &DeriveInput, fields: &Vec<AttributedField>) -> TokenStream {
    let transformed_fields = generate_struct_members(fields, true);

    let name = get_compressed_struct_name(input, false);
    let attributes = strip_compress_attribute(&input.attrs);
    let vis = &input.vis;

    quote_spanned! {input.span()=>
        #[allow(dead_code)]
        #(#attributes)*
        #vis struct #name {
            #(#transformed_fields)*
        }
    }
}

fn generate_compress_impl(input: &DeriveInput, fields: &Vec<AttributedField>, is_ordered: bool) -> TokenStream {
    let compress_name = get_compressed_struct_name(input, is_ordered);
    let name = &input.ident;

    let compress_initializers = fields.iter().filter_map(|f| {
        let name = f.field.ident.as_ref().unwrap();
        let ty = &f.field.ty;
        
        match f.tag {
            Some(Tag::Excluded) => None,
            Some(Tag::Frozen) => Some(quote_spanned! {f.field.span()=> #name: ::core::clone::Clone::clone(&self.#name), }),
            Some(Tag::Ordered { is_frozen, .. }) if is_frozen => Some(quote_spanned! {f.field.span()=> #name: ::core::clone::Clone::clone(&self.#name), }),
            Some(Tag::Ordered { .. }) | None => Some(quote_spanned! {f.field.span()=> #name: <#ty as ::compaq::Compress>::compress(&self.#name), })
        }
    });

    let decompress_initializers = fields.iter().map(|f| {
        let name = f.field.ident.as_ref().unwrap();
        let ty = &f.field.ty;
        
        match f.tag {
            Some(Tag::Excluded) => quote_spanned! {f.field.span()=> #name: ::core::default::Default::default(), },
            Some(Tag::Frozen) => quote_spanned! {f.field.span()=> #name: val.#name, },
            Some(Tag::Ordered { is_frozen, .. }) if is_frozen => quote_spanned! {f.field.span()=> #name: val.#name, },
            Some(Tag::Ordered { .. }) | None => quote_spanned! {f.field.span()=> #name: <#ty as ::compaq::Compress>::decompress(val.#name), }
        }
    });

    quote! {
        #[automatically_derived]
        impl ::compaq::Compress for #name {
            type Compressed = #compress_name;

            fn compress(&self) -> Self::Compressed {
                Self::Compressed {
                    #(#compress_initializers)*
                }
            }

            fn decompress(val: Self::Compressed) -> Self {
                Self {
                    #(#decompress_initializers)*
                }
            }
        }
    }
}

fn isolate_ordered_fields<'a>(fields: &'a Vec<AttributedField<'a>>) -> impl Iterator<Item = &'a AttributedField<'a>> {
    fields.iter().filter(move |f| matches!(f.tag, Some(Tag::Ordered { .. })))
}

fn generate_methods(input: &DeriveInput, fields: &Vec<AttributedField>) -> TokenStream {
    let name = &input.ident;
    let vis = &input.vis;
    let compressed_name = get_compressed_struct_name(input, false);

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
            if self.#name.len() != #policy_name.len() {
                return ::core::result::Result::Err(::compaq::CompaqError::DesynchronizedPolicy);
            }

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
            Some(Tag::Frozen) => Some(quote_spanned! {f.field.span()=> #name: ::core::clone::Clone::clone(&self.#name), }),
            Some(Tag::Ordered { .. }) => Some(quote_spanned! {f.field.span()=> #name, }),
            None => Some(quote_spanned! {f.field.span()=> #name: <#ty as ::compaq::Compress>::compress(&self.#name), }),
        }
    });

    let inflate_initializers = fields.iter().map(|f| {
        let name = f.field.ident.as_ref().unwrap();
        let ty = &f.field.ty;
        
        match f.tag {
            Some(Tag::Excluded) => quote_spanned! { f.field.span()=> #name: ::core::default::Default::default(), },
            Some(Tag::Frozen) => quote_spanned! {f.field.span()=> #name: self.#name, },
            Some(Tag::Ordered { .. }) => quote_spanned! {f.field.span()=> #name, },
            None => quote_spanned! {f.field.span()=> #name: <#ty as ::compaq::Compress>::decompress(self.#name), },
        }
    });
    
    quote! {
        #[automatically_derived]
        impl #name {
            /// Compresses the object into a smaller format.
            #vis fn deflate(&self #(#deflate_policy_parameters)*) -> ::compaq::Result<#compressed_name> {
                #(#deflate_ordered_logic)*

                ::core::result::Result::Ok(#compressed_name {
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
                    #(#inflate_initializers)*
                })
            }
        }
    }
}

fn strip_compress_attribute(attrs: &[Attribute]) -> Vec<Attribute> {
    let mut attrs = attrs.to_owned();
    let Some(index) = attrs.iter().position(|val| val.path().is_ident("derive")) else {
        return attrs;
    };
    let attr = attrs.remove(index);

    let args: Punctuated<Path, Token![,]> = attr.parse_args_with(Punctuated::<Path, Token![,]>::parse_terminated).unwrap();
    let filtered: Vec<Path> = args.into_iter().filter(|p| !p.is_ident(MACRO_STRING)).collect();

    attrs.insert(index, parse_quote! { #[derive(#(#filtered),*)] });
    attrs
}

#[proc_macro_derive(__SilenceErrors, attributes(exclude, freeze, order))]
pub fn derive(_input: proc_macro::TokenStream) -> proc_macro::TokenStream {
    TokenStream::new().into()
}

#[proc_macro_attribute]
pub fn compress(_attr: proc_macro::TokenStream, input: proc_macro::TokenStream) -> proc_macro::TokenStream {
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
    let compressed_struct = generate_compressed_struct(&input, &fields, has_ordered_member);
    let compress_impl = generate_compress_impl(&input, &fields, has_ordered_member);
    let methods = generate_methods(&input, &fields);
    let ordered_struct = if has_ordered_member { generate_ordered_struct(&input, &fields) } else { TokenStream::new() };

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