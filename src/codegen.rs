//! Helper macros for code generation

macro_rules! extract {
    ($data:ident, $offset:ident, bool) => {{
        let b0 = $data[$offset];
        $offset += 1;
        match b0 {
            0 => false,
            1 => true,
            _ => return Err($data),
        }
    }};
    ($data:ident, $offset:ident, u8) => {{
        let b0 = $data[$offset];
        $offset += 1;
        b0
    }};
    ($data:ident, $offset:ident, u16) => {{
        let [b0, b1] = [$data[$offset], $data[$offset + 1]];
        $offset += 2;
        u16::from_le_bytes([b0, b1])
    }};
    ($data:ident, $offset:ident, u32) => {{
        let [b0, b1, b2, b3] = [
            $data[$offset],
            $data[$offset + 1],
            $data[$offset + 2],
            $data[$offset + 3],
        ];
        $offset += 4;
        u32::from_le_bytes([b0, b1, b2, b3])
    }};
    ($data:ident, $offset:ident, i8) => {{
        let b0 = $data[$offset];
        $offset += 1;
        b0 as i8
    }};
    ($data:ident, $offset:ident, i16) => {{
        let [b0, b1] = [$data[$offset], $data[$offset + 1]];
        $offset += 2;
        i16::from_le_bytes([b0, b1])
    }};
    ($data:ident, $offset:ident, i32) => {{
        let [b0, b1, b2, b3] = [
            $data[$offset],
            $data[$offset + 1],
            $data[$offset + 2],
            $data[$offset + 3],
        ];
        $offset += 4;
        i32::from_le_bytes([b0, b1, b2, b3])
    }};
    ($data:ident, $offset:ident, f32) => {{
        let [b0, b1, b2, b3] = [
            $data[$offset],
            $data[$offset + 1],
            $data[$offset + 2],
            $data[$offset + 3],
        ];
        $offset += 4;
        f32::from_le_bytes([b0, b1, b2, b3])
    }};
    ($data:ident, $offset:ident, Month) => {{
        let b0 = $data[$offset];
        $offset += 1;
        if let Ok(month) = Month::try_from_bcd(b0) {
            month
        } else {
            return Err($data);
        }
    }};
    ($data:ident, $offset:ident, Date) => {{
        let b0 = $data[$offset];
        $offset += 1;
        if let Ok(date) = Date::try_from_bcd(b0) {
            date
        } else {
            return Err($data);
        }
    }};
}

macro_rules! inject {
    ($name:ident, $data:ident, $offset:ident, bool) => {
        $data[$offset] = $name as u8;
        $offset += 1;
    };
    ($name:ident, $data:ident, $offset:ident, u8) => {
        $data[$offset] = $name;
        $offset += 1;
    };
    ($name:ident, $data:ident, $offset:ident, u16) => {
        [$data[$offset], $data[$offset + 1]] = $name.to_le_bytes();
        $offset += 2;
    };
    ($name:ident, $data:ident, $offset:ident, u32) => {
        [
            $data[$offset],
            $data[$offset + 1],
            $data[$offset + 2],
            $data[$offset + 3],
        ] = $name.to_le_bytes();
        $offset += 4;
    };
    ($name:ident, $data:ident, $offset:ident, i8) => {
        $data[$offset] = $name as u8;
        $offset += 1;
    };
    ($name:ident, $data:ident, $offset:ident, i16) => {
        [$data[$offset], $data[$offset + 1]] = $name.to_le_bytes();
        $offset += 2;
    };
    ($name:ident, $data:ident, $offset:ident, i32) => {
        [
            $data[$offset],
            $data[$offset + 1],
            $data[$offset + 2],
            $data[$offset + 3],
        ] = $name.to_le_bytes();
        $offset += 4;
    };
    ($name:ident, $data:ident, $offset:ident, f32) => {
        [
            $data[$offset],
            $data[$offset + 1],
            $data[$offset + 2],
            $data[$offset + 3],
        ] = $name.to_le_bytes();
        $offset += 4;
    };
    ($name:ident, $data:ident, $offset:ident, Month) => {
        $data[$offset] = $name.bcd();
        $offset += 1;
    };
    ($name:ident, $data:ident, $offset:ident, Date) => {
        $data[$offset] = $name.bcd();
        $offset += 1;
    };
}

// Keeping this around for the sake of leaving room for more complex serialization logic
/* macro_rules! build_injector {
    ({ $($stmts:stmt)* } $offset:expr, $data:ident @) => {
        $($stmts)*
    };
    ({ $($stmts:stmt)* } $offset:expr, $data:ident @ $name:ident, bool; $($tail:tt)*) => {
        build_injector!(
            { $($stmts)*
            $data[$offset] = $name as u8; }
            $offset + 1, $data
            @ $($tail)*
        )
    };
    ({ $($stmts:stmt)* } $offset:expr, $data:ident @ $name:ident, u8; $($tail:tt)*) => {
        build_injector!(
            { $($stmts)*
            $data[$offset] = $name; }
            $offset + 1, $data
            @ $($tail)*
        )
    };
    ({ $($stmts:stmt)* } $offset:expr, $data:ident @ $name:ident, u16; $($tail:tt)*) => {
        build_injector!(
            { $($stmts)*
            [$data[$offset], $data[$offset + 1]] = $name.to_le_bytes(); }
            $offset + 2, $data
            @ $($tail)*
        )
    };
    ({ $($stmts:stmt)* } $offset:expr, $data:ident @ $name:ident, u32; $($tail:tt)*) => {
        build_injector!(
            { $($stmts)*
            [$data[$offset], $data[$offset + 1], $data[$offset + 2], $data[$offset + 3]] = $name.to_le_bytes(); }
            $offset + 4, $data
            @ $($tail)*
        )
    };
    ({ $($stmts:stmt)* } $offset:expr, $data:ident @ $name:ident, i8; $($tail:tt)*) => {
        build_injector!(
            { $($stmts)*
            $data[$offset] = $name as u8; }
            $offset + 1, $data
            @ $($tail)*
        )
    };
    ({ $($stmts:stmt)* } $offset:expr, $data:ident @ $name:ident, i16; $($tail:tt)*) => {
        build_injector!(
            { $($stmts)*
            [$data[$offset], $data[$offset + 1]] = $name.to_le_bytes(); }
            $offset + 2, $data
            @ $($tail)*
        )
    };
    ({ $($stmts:stmt)* } $offset:expr, $data:ident @ $name:ident, i32; $($tail:tt)*) => {
        build_injector!(
            { $($stmts)*
            [$data[$offset], $data[$offset + 1], $data[$offset + 2], $data[$offset + 3]] = $name.to_le_bytes(); }
            $offset + 4, $data
            @ $($tail)*
        )
    };
    ({ $($stmts:stmt)* } $offset:expr, $data:ident @ $name:ident, f32; $($tail:tt)*) => {
        build_injector!(
            { $($stmts)*
            [$data[$offset], $data[$offset + 1], $data[$offset + 2], $data[$offset + 3]] = $name.to_le_bytes(); }
            $offset + 4, $data
            @ $($tail)*
        )
    };
    ({ $($stmts:stmt)* } $offset:expr, $data:ident @ $name:ident, Month; $($tail:tt)*) => {
        build_injector!(
            { $($stmts)*
            $data[$offset] = $name.bcd(); }
            $offset + 1, $data
            @ $($tail)*
        )
    };
    ({ $($stmts:stmt)* } $offset:expr, $data:ident @ $name:ident, Date; $($tail:tt)*) => {
        build_injector!(
            { $($stmts)*
            $data[$offset] = $name.bcd(); }
            $offset + 1, $data
            @ $($tail)*
        )
    };
} */

macro_rules! incrementor {
    ($value:ident, bool) => {
        !$value
    };
    ($value:ident, u8) => {
        $value.saturating_add(1)
    };
    ($value:ident, u16) => {
        $value.saturating_add(1)
    };
    ($value:ident, u32) => {
        $value.saturating_add(1)
    };
    ($value:ident, i8) => {
        $value.saturating_add(1)
    };
    ($value:ident, i16) => {
        $value.saturating_add(1)
    };
    ($value:ident, i32) => {
        $value.saturating_add(1)
    };
    ($value:ident, f32) => {
        $value + 0.25
    };
    ($value:ident, Month) => {
        $value.next()
    };
    ($value:ident, Date) => {
        $value.next()
    };
    ($value:ident, Duty) => {
        $value.next()
    };
}

macro_rules! decrementor {
    ($value:ident, bool) => {
        !$value
    };
    ($value:ident, u8) => {
        $value.saturating_sub(1)
    };
    ($value:ident, u16) => {
        $value.saturating_sub(1)
    };
    ($value:ident, u32) => {
        $value.saturating_sub(1)
    };
    ($value:ident, i8) => {
        $value.saturating_sub(1)
    };
    ($value:ident, i16) => {
        $value.saturating_sub(1)
    };
    ($value:ident, i32) => {
        $value.saturating_sub(1)
    };
    ($value:ident, f32) => {
        $value - 0.25
    };
    ($value:ident, Month) => {
        $value.prev()
    };
    ($value:ident, Date) => {
        $value.prev()
    };
    ($value:ident, Duty) => {
        $value.prev()
    };
}

macro_rules! draw_method {
    ($value:expr, $page:ident, bool) => {
        $page.write(if $value { b"      On" } else { b"     Off" })
    };
    ($value:expr, $page:ident, u8) => {
        $page.skip(2).uint($value as u16)
    };
    ($value:expr, $page:ident, u16) => {
        $page.skip(2).uint($value)
    };
    ($value:expr, $page:ident, u32) => {
        $page.skip(2).uint($value as u16)
    };
    ($value:expr, $page:ident, i8) => {
        $page.skip(2).sint($value as i16)
    };
    ($value:expr, $page:ident, i16) => {
        $page.skip(2).sint($value)
    };
    ($value:expr, $page:ident, i32) => {
        $page.skip(2).sint($value as i16)
    };
    ($value:expr, $page:ident, f32) => {
        $page.decimal($value).byte(b'F')
    };
    ($value:expr, $page:ident, Month) => {
        $page.skip(5).write($value.abbrev().as_bytes())
    };
    ($value:expr, $page:ident, Date) => {{
        $page.skip(4).hexit2($value.bcd()).write($value.suffix())
    }};
    ($value:expr, $page:ident, Duty) => {
        $page.skip(2).uint($value.0)
    };
}

macro_rules! range_hint {
    (bool) => {
        b"R=[T,F]     S=toggle"
    };
    (u8) => {
        b"R=[0,255]        S=1"
    };
    (u16) => {
        b"R=[0,65535]      S=1"
    };
    (u32) => {
        b"R=[0,~4.3B]      S=1"
    };
    (i8) => {
        b"R=[-128,127]     S=1"
    };
    (i16) => {
        b"R=[-32768,32767] S=1"
    };
    (i32) => {
        b"R=[-2.15B,2.15B] S=1"
    };
    (f32) => {
        b"R=[-999,999]  S=0.25"
    };
    (Month) => {
        b"R=[Jan,Dec] S=1month"
    };
    (Date) => {
        b"R=[1,[28-31]] S=1day"
    };
    (Duty) => {
        b"R=[0,256]        S=1"
    };
}

macro_rules! build_setter {
    ($kind:ident match $buffer:ident {
        $($arm_pat:pat => $arm_block:block),*
     } $self_:ident @) => {
        match $buffer {
            $($arm_pat => $arm_block),*
        }
     };
    ($kind:ident match $buffer:ident {
        $($arm_pat:pat => $arm_block:block),*
     } $self_:ident @ $disp_a:ident, $field_a:ident, Month; $disp_b:ident, $field_b:ident, Date; $($tail:tt)*) => {
        crate::codegen::build_setter!($kind match $buffer {
            $($arm_pat => $arm_block,)*
            $kind::$disp_a(x) => {
                $self_.$field_a = x;
                $self_.$field_b = $self_.$field_b.with_limit(x.length(false));
            },
            $kind::$disp_b(x) => {
                $self_.$field_b = x;
            }
        }
        $self_ @ $($tail)*)
    };
    ($kind:ident match $buffer:ident {
        $($arm_pat:pat => $arm_block:block),*
     } $self_:ident @ $disp_name:ident, $field_name:ident, $type:ty; $($tail:tt)*) => {
        crate::codegen::build_setter!($kind match $buffer {
            $($arm_pat => $arm_block,)*
            $kind::$disp_name(x) => {
                $self_.$field_name = x;
            }
        }
        $self_ @ $($tail)*)
    };
}

macro_rules! interactive {
   (
        $(#[$meta:meta])*
        $vis:vis struct $name:ident {
            $(
                $(#[$field_meta:meta])*
                $field_vis:vis $field_name:ident as $disp_name:ident : $field_type:tt = $default:expr,
            )*
        }
        exit = $exit_str:literal;
        info = $info_str:literal;
        $buffer_name:ident
    ) => {
        $(#[$meta])*
        $vis struct $name {
            $(
                $(#[$field_meta])*
                $field_vis $field_name : $field_type,
            )*
        }

        impl $name {
            const DEFAULT: Self = Self {
                $($field_name : $default,)*
            };

            const FIELD_COUNT: u8 = [$(stringify!($field_name)),*].len() as u8;

            const NAMES: [[u8; 18]; Self::FIELD_COUNT as usize + 1] = [crate::utils::pad_bytes($exit_str), $(crate::utils::pad_bytes(stringify!($disp_name).as_bytes())),*];

            const fn get_buffer(&self, index: u8) -> Option<$buffer_name> {
                if index == 0 { return None; }
                match index - 1 {
                    $(
                        ${index()} => Some($buffer_name::$disp_name(self.$field_name)),
                    )*
                    _ => None
                }
            }

            const fn set_buffer(&mut self, buffer: $buffer_name) {
                crate::codegen::build_setter!($buffer_name match buffer {} self @ $($disp_name, $field_name, $field_type;)*)
            }
        }

        #[derive(Clone, Copy)]
        enum $buffer_name {
            $($disp_name($field_type),)*
        }

        impl $buffer_name {
            const fn adjust(&mut self, click: Click) {
                *self = match click {
                    Click::CW => match *self {
                        $(
                            Self::$disp_name(x) => Self::$disp_name(crate::codegen::incrementor!(x, $field_type))
                        ),*
                    },
                    Click::CCW => match *self {
                        $(
                            Self::$disp_name(x) => Self::$disp_name(crate::codegen::decrementor!(x, $field_type))
                        ),*
                    },
                }
            }

            const fn name(&self) -> &[u8; 18] {
                match self {
                    $(
                        Self::$disp_name(_) => &$name::NAMES[${index()} + 1]
                    ),*
                }
            }

            const fn draw(&self, page: PageBuilder) -> PageBuilder {
                match self {
                    $(
                        Self::$disp_name(x) => crate::codegen::draw_method!(*x, page, $field_type)
                    ),*
                }
            }

            const fn range_hint(&self) -> &[u8; 20] {
                match self {
                    $(
                        Self::$disp_name(_) => crate::codegen::range_hint!($field_type)
                    ),*
                }
            }

            const fn generate_edit_page(&self) -> PageData {
                let page = PageBuilder::new()
                    .byte(b'[')
                    .write(self.name())
                    .byte(b']')
                    .end_line()
                    .skip(2);
                self
                    .draw(page)
                    .end_line()
                    .write(self.range_hint())
                    .write($info_str)
                    .finish()
            }
        }
    };
}

macro_rules! portable {
    (
        $(#[$meta:meta])*
        $vis:vis struct $name:ident {
            $(
                $(#[$field_meta:meta])*
                $field_vis:vis $field_name:ident as $disp_name:ident : $field_type:tt = $default:expr,
            )*
        }
        exit = $exit_str:literal;
        info = $info_str:literal;
        $buffer_name:ident
    ) => {
        crate::codegen::interactive!(
            $(#[$meta])*
            $vis struct $name {
                $(
                    $(#[$field_meta])*
                    $field_vis $field_name as $disp_name : $field_type = $default,
                )*
            }
            exit = $exit_str;
            info = $info_str;
            $buffer_name
        );

        impl $name {
            const SIGNATURE: u32 = {
                let bytes = concat!($(stringify!($field_type)),*).as_bytes();
                let mut sig: u32 = 0;
                let mut i = 0;
                while i < bytes.len() {
                    let byte = bytes[i];
                    let un = (byte >> 4) as u32;
                    let ln = (byte & 0xf) as u32;
                    sig ^= (1 << (un + 16)) | (1 << ln);
                    sig = sig.rotate_left(1);
                    i += 1;
                }
                sig
            };

            #[expect(unused_assignments, reason = "macro expansion leaves trailing offset increment")]
            #[inline(never)]
            const fn from_data(data: [u8; 56]) -> Result<Self, [u8; 56]> {
                let data_sig = u32::from_le_bytes([data[52], data[53], data[54], data[55]]);
                if data_sig == Self::SIGNATURE {
                    let mut offset = 0;
                    Ok(Self{$($field_name: crate::codegen::extract!(data, offset, $field_type)),*})
                } else {
                    Err(data)
                }
            }

            #[expect(unused_assignments, reason = "macro expansion leaves trailing offset increment")]
            #[inline(never)]
            const fn into_data(self) -> [u8;56] {
                let Self {$($field_name),*} = self;
                let mut data = [0;56];
                let mut offset = 0;
                $(crate::codegen::inject!($field_name, data, offset, $field_type);)*
                //build_injector!({} 0, data @ $($field_name, $field_type;)*);
                [data[52], data[53], data[54], data[55]] = Self::SIGNATURE.to_le_bytes();
                data
            }
        }

        impl From<$name> for [u8; 56] {
            fn from(value: $name) -> Self {
                value.into_data()
            }
        }

        impl From<[u8; 56]> for $name {
            fn from(value: [u8; 56]) -> Self {
                Self::from_data(value).unwrap_or(Self::DEFAULT)
            }
        }
    };
}

macro_rules! build_revolver {
    ($type:ident match $value:ident {
        $($in_names:pat => $out_names:expr),*
     } @forward $name_a:ident @ $name_b:ident) => {
        match $value {
            $($in_names => $out_names,)*
            $type::$name_a => $type::$name_b
        }
    };
    ($type:ident match $value:ident {
        $($in_names:pat => $out_names:expr),*
     } @reverse $name_a:ident @ $name_b:ident) => {
        match $value {
            $($in_names => $out_names,)*
            $type::$name_b => $type::$name_a
        }
    };
    ($type:ident match $value:ident {
        $($in_names:pat => $out_names:expr),*
     } @forward $name_a:ident $name_b:ident $($tail:tt)*) => {
        crate::codegen::build_revolver!($type match $value {
            $($in_names => $out_names,)*
            $type::$name_a => $type::$name_b
        } @forward $name_b $($tail)*)
    };
    ($type:ident match $value:ident {
        $($in_names:pat => $out_names:expr),*
     } @reverse $name_a:ident $name_b:ident $($tail:tt)*) => {
        crate::codegen::build_revolver!($type match $value {
            $($in_names => $out_names,)*
            $type::$name_b => $type::$name_a
        } @reverse $name_b $($tail)*)
    };
    ($type:ident match $value:ident {
        $($in_names:pat => $out_names:expr),*
     } @forward @ $name_a:ident $name_b:ident $($tail:tt)*) => {
        crate::codegen::build_revolver!($type match $value {
            $($in_names => $out_names,)*
            $type::$name_a => $type::$name_b
        } @forward $name_b $($tail)* @ $name_a)
    };
    ($type:ident match $value:ident {
        $($in_names:pat => $out_names:expr),*
     } @reverse @ $name_a:ident $name_b:ident $($tail:tt)*) => {
        crate::codegen::build_revolver!($type match $value {
            $($in_names => $out_names,)*
            $type::$name_b => $type::$name_a
        } @reverse $name_b $($tail)* @ $name_a)
    };
}

macro_rules! revolving_enum {
    (
        $(#[$meta:meta])*
        $vis:vis enum $name:ident {
            $($var_vis:vis $var_name:ident),* $(,)?
        }
    ) => {
        $(#[$meta])*
        $vis enum $name {
            $($var_vis $var_name,)*
        }

        impl $name {
            const fn next(self) -> Self {
                crate::codegen::build_revolver!($name match self {} @forward @ $($var_name)*)
            }

            const fn prev(self) -> Self {
                crate::codegen::build_revolver!($name match self {} @reverse @ $($var_name)*)
            }
        }
    };
}

pub(crate) use {
    build_revolver, build_setter, decrementor, draw_method, extract, incrementor, inject,
    interactive, portable, range_hint, revolving_enum,
};
