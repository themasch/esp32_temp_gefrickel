struct WriteBuffer<'a> {
    buf: &'a mut [u8],
    written: usize,
}

impl<'a> core::fmt::Write for WriteBuffer<'a> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        if self.written > self.buf.len() {
            return Err(core::fmt::Error);
        }

        let rem = &mut self.buf[self.written..];
        let raw_s = s.as_bytes();
        let num = core::cmp::min(raw_s.len(), rem.len());

        rem[..num].copy_from_slice(&raw_s[..num]);
        self.written += raw_s.len();

        if num < raw_s.len() {
            Err(core::fmt::Error)
        } else {
            Ok(())
        }
    }
}

pub fn format<'a>(
    buffer: &'a mut [u8],
    a: core::fmt::Arguments,
) -> Result<&'a str, core::fmt::Error> {
    let mut write_buffer = WriteBuffer {
        buf: buffer,
        written: 0,
    };
    core::fmt::write(&mut write_buffer, a)?;
    core::str::from_utf8(&write_buffer.buf[..write_buffer.written]).map_err(|_| core::fmt::Error)
}
