//! Multi-bus poller built on `mio::Poll`.

use std::os::fd::{BorrowedFd, RawFd};
use std::time::Duration;

use mio::unix::SourceFd;
use mio::{Events, Interest, Poll, Token};

use super::TransportError;

/// Registers multiple bus fds and waits for any to become readable within a
/// deadline. Buses with no `raw_fd` cannot be registered and must be polled
/// out-of-band.
pub struct BusPoller {
    poll: Poll,
    events: Events,
}

impl BusPoller {
    /// Construct a poller with capacity for `n` simultaneous ready buses.
    pub fn with_capacity(n: usize) -> Result<Self, TransportError> {
        let poll = Poll::new().map_err(TransportError::Io)?;
        Ok(Self {
            poll,
            events: Events::with_capacity(n.max(1)),
        })
    }

    /// Register a fd under a token.
    pub fn register(&self, token: Token, fd: RawFd) -> Result<(), TransportError> {
        // SAFETY: BorrowedFd is constructed for the duration of register;
        // the caller is responsible for keeping the fd alive while registered.
        let mut src = SourceFd(&fd);
        self.poll
            .registry()
            .register(&mut src, token, Interest::READABLE)
            .map_err(TransportError::Io)?;
        // BorrowedFd is only used for its lifetime guarantee; we drop it now.
        let _ = unsafe { BorrowedFd::borrow_raw(fd) };
        Ok(())
    }

    /// Deregister a fd; call when the bus is going away.
    pub fn deregister(&self, fd: RawFd) -> Result<(), TransportError> {
        let mut src = SourceFd(&fd);
        self.poll
            .registry()
            .deregister(&mut src)
            .map_err(TransportError::Io)
    }

    /// Block up to `deadline` waiting for any registered fd to become readable.
    /// Returns the tokens of the ready fds in arrival order.
    pub fn wait(&mut self, deadline: Duration) -> Result<Vec<Token>, TransportError> {
        match self.poll.poll(&mut self.events, Some(deadline)) {
            Ok(()) => {}
            Err(e) if e.kind() == std::io::ErrorKind::Interrupted => {
                // EINTR: treat as a spurious wake-up and return empty.
                return Ok(Vec::new());
            }
            Err(e) => return Err(TransportError::Io(e)),
        }
        Ok(self.events.iter().map(|ev| ev.token()).collect())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::os::fd::AsRawFd;
    use std::time::Instant;

    fn make_pipe() -> (std::fs::File, std::fs::File) {
        // Use a simple unix pipe for fd-readable tests.
        let mut pipefd = [0i32; 2];
        // SAFETY: standard pipe() with a valid output array.
        let rc = unsafe { libc::pipe(pipefd.as_mut_ptr()) };
        assert!(rc == 0);
        unsafe {
            (
                std::fs::File::from_raw_fd_unchecked(pipefd[0]),
                std::fs::File::from_raw_fd_unchecked(pipefd[1]),
            )
        }
    }

    trait FromRawFdUnchecked {
        unsafe fn from_raw_fd_unchecked(fd: i32) -> Self;
    }
    impl FromRawFdUnchecked for std::fs::File {
        unsafe fn from_raw_fd_unchecked(fd: i32) -> Self {
            use std::os::fd::FromRawFd;
            std::fs::File::from_raw_fd(fd)
        }
    }

    #[test]
    fn quiet_buses_deadline_expires() {
        let mut p = BusPoller::with_capacity(4).unwrap();
        let (r, _w) = make_pipe();
        p.register(Token(0), r.as_raw_fd()).unwrap();
        let t0 = Instant::now();
        let tokens = p.wait(Duration::from_millis(5)).unwrap();
        let elapsed = t0.elapsed();
        assert!(tokens.is_empty(), "tokens: {tokens:?}");
        assert!(
            elapsed < Duration::from_millis(50),
            "took too long: {elapsed:?}"
        );
    }

    #[test]
    fn wake_on_readable_pipe() {
        use std::io::Write;
        let mut p = BusPoller::with_capacity(4).unwrap();
        let (r, mut w) = make_pipe();
        p.register(Token(7), r.as_raw_fd()).unwrap();
        w.write_all(b"x").unwrap();
        let tokens = p.wait(Duration::from_millis(100)).unwrap();
        assert!(tokens.contains(&Token(7)), "tokens: {tokens:?}");
    }
}
