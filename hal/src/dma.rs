//! # Direct Memory Access
#![allow(dead_code)]

use core::{
    marker::PhantomData,
    sync::atomic::{compiler_fence, Ordering},
};
use embedded_dma::{ReadBuffer, WriteBuffer};

#[derive(Debug)]
#[non_exhaustive]
pub enum Error {
    Overrun,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Event {
    HalfTransfer,
    TransferComplete,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Half {
    First,
    Second,
}

pub struct CircBuffer<BUFFER, PAYLOAD>
where
    BUFFER: 'static,
{
    buffer: &'static mut [BUFFER; 2],
    payload: PAYLOAD,
    readable_half: Half,
}

impl<BUFFER, PAYLOAD> CircBuffer<BUFFER, PAYLOAD>
where
    &'static mut [BUFFER; 2]: WriteBuffer,
    BUFFER: 'static,
{
    pub(crate) fn new(buf: &'static mut [BUFFER; 2], payload: PAYLOAD) -> Self {
        CircBuffer {
            buffer: buf,
            payload,
            readable_half: Half::Second,
        }
    }
}

pub trait DmaExt {
    type Channels;

    fn split(self) -> Self::Channels;
}

pub trait TransferPayload {
    fn start(&mut self);
    fn stop(&mut self);
}

pub struct Transfer<MODE, BUFFER, PAYLOAD>
where
    PAYLOAD: TransferPayload,
{
    _mode: PhantomData<MODE>,
    buffer: BUFFER,
    payload: PAYLOAD,
}

impl<BUFFER, PAYLOAD> Transfer<R, BUFFER, PAYLOAD>
where
    PAYLOAD: TransferPayload,
{
    pub(crate) fn r(buffer: BUFFER, payload: PAYLOAD) -> Self {
        Transfer {
            _mode: PhantomData,
            buffer,
            payload,
        }
    }
}

impl<BUFFER, PAYLOAD> Transfer<W, BUFFER, PAYLOAD>
where
    PAYLOAD: TransferPayload,
{
    pub(crate) fn w(buffer: BUFFER, payload: PAYLOAD) -> Self {
        Transfer {
            _mode: PhantomData,
            buffer,
            payload,
        }
    }
}

impl<MODE, BUFFER, PAYLOAD> Drop for Transfer<MODE, BUFFER, PAYLOAD>
where
    PAYLOAD: TransferPayload,
{
    fn drop(&mut self) {
        self.payload.stop();
        compiler_fence(Ordering::SeqCst);
    }
}

/// Read transfer
pub struct R;

/// Write transfer
pub struct W;

macro_rules! dma {
    ($($DMAX:ident: ($dmaX:ident, {
        $($CX:ident: (
            $cfgrX:ident, $CFGRX:ident,
            $cntrX:ident, $CNTRX:ident,
            $paddrX:ident, $PADDRX:ident,
            $maddrX:ident, $MADDRX:ident,
            $intfrX:ident,
            $intfcrX:ident, $INTFCRX:ident,
            $htifX:ident,
            $tcifX:ident,
            $chtifX:ident,
            $ctcifX:ident,
            $cgifX:ident
        ),)+
    }),)+) => {
        $(
            pub mod $dmaX {
                use core::{sync::atomic::{self, Ordering}, ptr, mem, convert::TryFrom};

                use crate::pac::{RCC, $DMAX, dma2};

                use crate::dma::{CircBuffer, DmaExt, Error, Event, Half, Transfer, W, RxDma, TxDma, RxTxDma, TransferPayload};
                use crate::rcc::Enable;

                #[allow(clippy::manual_non_exhaustive)]
                pub struct Channels((), $(pub $CX),+);

                $(
                    /// A singleton that represents a single DMAx channel (channel X in this case)
                    ///
                    /// This singleton has exclusive access to the registers of the DMAx channel X
                    pub struct $CX { _0: () }

                    impl $CX {
                        /// Associated peripheral `address`
                        ///
                        /// `inc` indicates whether the address will be incremented after every byte transfer
                        pub fn set_peripheral_address(&mut self, address: u32, inc: bool) {
                            self.paddr().write(|w| unsafe { w.pa().bits(address) } );
                            self.cfgr().modify(|_, w| w.pinc().bit(inc) );
                        }

                        /// `address` where from/to data will be read/write
                        ///
                        /// `inc` indicates whether the address will be incremented after every byte transfer
                        pub fn set_memory_address(&mut self, address: u32, inc: bool) {
                            self.maddr().write(|w| unsafe { w.ma().bits(address) } );
                            self.cfgr().modify(|_, w| w.minc().bit(inc) );
                        }

                        /// Number of bytes to transfer
                        pub fn set_transfer_length(&mut self, len: usize) {
                            self.cntr().write(|w| unsafe{w.ndt().bits(u16::try_from(len).unwrap())});
                        }

                        /// Starts the DMA transfer
                        pub fn start(&mut self) {
                            self.cfgr().modify(|_, w| w.en().set_bit() );
                        }

                        /// Stops the DMA transfer
                        pub fn stop(&mut self) {
                            self.intfcr().write(|w| w.$cgifX().set_bit());
                            self.cfgr().modify(|_, w| w.en().clear_bit() );
                        }

                        /// Returns `true` if there's a transfer in progress
                        pub fn in_progress(&self) -> bool {
                            self.intfr().$tcifX().bit_is_clear()
                        }
                    }

                    impl $CX {
                        pub fn listen(&mut self, event: Event) {
                            match event {
                                Event::HalfTransfer => self.cfgr().modify(|_, w| w.htie().set_bit()),
                                Event::TransferComplete => {
                                    self.cfgr().modify(|_, w| w.tcie().set_bit())
                                }
                            }
                        }

                        pub fn unlisten(&mut self, event: Event) {
                            match event {
                                Event::HalfTransfer => {
                                    self.cfgr().modify(|_, w| w.htie().clear_bit())
                                },
                                Event::TransferComplete => {
                                    self.cfgr().modify(|_, w| w.tcie().clear_bit())
                                }
                            }
                        }

                        // TODO the PAC crate doesn't support grouped Channels now. wait for the PAC
                        // pub fn ch(&mut self) -> &dma2::CH {
                        //     unsafe { &(*$DMAX::ptr()).$chX }
                        // }

                        fn cfgr(&self) -> &dma2::$CFGRX {
                            unsafe { &(*$DMAX::ptr()).$cfgrX }
                        }

                        fn cntr(&self) -> &dma2::$CNTRX {
                            unsafe { &(*$DMAX::ptr()).$cntrX }
                        }

                        fn maddr(&self) -> &dma2::$MADDRX {
                            unsafe { &(*$DMAX::ptr()).$maddrX }
                        }

                        fn paddr(&self) -> &dma2::$PADDRX {
                            unsafe { &(*$DMAX::ptr()).$paddrX }
                        }

                        pub fn intfr(&self) -> dma2::$intfrX::R {
                            // NOTE(unsafe) atomic read with no side effects
                            unsafe { (*$DMAX::ptr()).$intfrX.read() }
                        }

                        pub fn intfcr(&self) -> &dma2::$INTFCRX {
                            unsafe { &(*$DMAX::ptr()).$intfcrX }
                        }

                        pub fn get_ndtr(&self) -> u32 {
                            // NOTE(unsafe) atomic read with no side effects
                            unsafe { &(*$DMAX::ptr())}.$cntrX.read().bits()
                        }
                    }

                    impl<B, PAYLOAD> CircBuffer<B, RxDma<PAYLOAD, $CX>>
                    where
                        RxDma<PAYLOAD, $CX>: TransferPayload,
                    {
                        /// Peeks into the readable half of the buffer
                        pub fn peek<R, F>(&mut self, f: F) -> Result<R, Error>
                            where
                            F: FnOnce(&B, Half) -> R,
                        {
                            let half_being_read = self.readable_half()?;

                            let buf = match half_being_read {
                                Half::First => &self.buffer[0],
                                Half::Second => &self.buffer[1],
                            };

                            // XXX does this need a compiler barrier?
                            let ret = f(buf, half_being_read);


                            let intfr = self.payload.channel.intfr();
                            let first_half_is_done = intfr.$htifX().bit_is_set();
                            let second_half_is_done = intfr.$tcifX().bit_is_set();

                            if (half_being_read == Half::First && second_half_is_done) ||
                                (half_being_read == Half::Second && first_half_is_done) {
                                Err(Error::Overrun)
                            } else {
                                Ok(ret)
                            }
                        }

                        /// Returns the `Half` of the buffer that can be read
                        pub fn readable_half(&mut self) -> Result<Half, Error> {
                            let intfr = self.payload.channel.intfr();
                            let first_half_is_done = intfr.$htifX().bit_is_set();
                            let second_half_is_done = intfr.$tcifX().bit_is_set();

                            if first_half_is_done && second_half_is_done {
                                return Err(Error::Overrun);
                            }

                            let last_read_half = self.readable_half;

                            Ok(match last_read_half {
                                Half::First => {
                                    if second_half_is_done {
                                        self.payload.channel.intfcr().write(|w| w.$ctcifX().set_bit());

                                        self.readable_half = Half::Second;
                                        Half::Second
                                    } else {
                                        last_read_half
                                    }
                                }
                                Half::Second => {
                                    if first_half_is_done {
                                        self.payload.channel.intfcr().write(|w| w.$chtifX().set_bit());

                                        self.readable_half = Half::First;
                                        Half::First
                                    } else {
                                        last_read_half
                                    }
                                }
                            })
                        }

                        /// Stops the transfer and returns the underlying buffer and RxDma
                        pub fn stop(mut self) -> (&'static mut [B; 2], RxDma<PAYLOAD, $CX>) {
                            self.payload.stop();

                            (self.buffer, self.payload)
                        }
                    }

                    impl<BUFFER, PAYLOAD, MODE> Transfer<MODE, BUFFER, RxDma<PAYLOAD, $CX>>
                    where
                        RxDma<PAYLOAD, $CX>: TransferPayload,
                    {
                        pub fn is_done(&self) -> bool {
                            !self.payload.channel.in_progress()
                        }

                        pub fn wait(mut self) -> (BUFFER, RxDma<PAYLOAD, $CX>) {
                            while !self.is_done() {}

                            atomic::compiler_fence(Ordering::Acquire);

                            self.payload.stop();

                            // we need a read here to make the Acquire fence effective
                            // we do *not* need this if `dma.stop` does a RMW operation
                            unsafe { ptr::read_volatile(&0); }

                            // we need a fence here for the same reason we need one in `Transfer.wait`
                            atomic::compiler_fence(Ordering::Acquire);

                            // `Transfer` needs to have a `Drop` implementation, because we accept
                            // managed buffers that can free their memory on drop. Because of that
                            // we can't move out of the `Transfer`'s fields, so we use `ptr::read`
                            // and `mem::forget`.
                            //
                            // NOTE(unsafe) There is no panic branch between getting the resources
                            // and forgetting `self`.
                            unsafe {
                                let buffer = ptr::read(&self.buffer);
                                let payload = ptr::read(&self.payload);
                                mem::forget(self);
                                (buffer, payload)
                            }
                        }
                    }

                    impl<BUFFER, PAYLOAD, MODE> Transfer<MODE, BUFFER, TxDma<PAYLOAD, $CX>>
                    where
                        TxDma<PAYLOAD, $CX>: TransferPayload,
                    {
                        pub fn is_done(&self) -> bool {
                            !self.payload.channel.in_progress()
                        }

                        pub fn wait(mut self) -> (BUFFER, TxDma<PAYLOAD, $CX>) {
                            while !self.is_done() {}

                            atomic::compiler_fence(Ordering::Acquire);

                            self.payload.stop();

                            // we need a read here to make the Acquire fence effective
                            // we do *not* need this if `dma.stop` does a RMW operation
                            unsafe { ptr::read_volatile(&0); }

                            // we need a fence here for the same reason we need one in `Transfer.wait`
                            atomic::compiler_fence(Ordering::Acquire);

                            // `Transfer` needs to have a `Drop` implementation, because we accept
                            // managed buffers that can free their memory on drop. Because of that
                            // we can't move out of the `Transfer`'s fields, so we use `ptr::read`
                            // and `mem::forget`.
                            //
                            // NOTE(unsafe) There is no panic branch between getting the resources
                            // and forgetting `self`.
                            unsafe {
                                let buffer = ptr::read(&self.buffer);
                                let payload = ptr::read(&self.payload);
                                mem::forget(self);
                                (buffer, payload)
                            }
                        }
                    }

                    impl<BUFFER, PAYLOAD, MODE, TXC> Transfer<MODE, BUFFER, RxTxDma<PAYLOAD, $CX, TXC>>
                    where
                        RxTxDma<PAYLOAD, $CX, TXC>: TransferPayload,
                    {
                        pub fn is_done(&self) -> bool {
                            !self.payload.rxchannel.in_progress()
                        }

                        pub fn wait(mut self) -> (BUFFER, RxTxDma<PAYLOAD, $CX, TXC>) {
                            while !self.is_done() {}

                            atomic::compiler_fence(Ordering::Acquire);

                            self.payload.stop();

                            // we need a read here to make the Acquire fence effective
                            // we do *not* need this if `dma.stop` does a RMW operation
                            unsafe { ptr::read_volatile(&0); }

                            // we need a fence here for the same reason we need one in `Transfer.wait`
                            atomic::compiler_fence(Ordering::Acquire);

                            // `Transfer` needs to have a `Drop` implementation, because we accept
                            // managed buffers that can free their memory on drop. Because of that
                            // we can't move out of the `Transfer`'s fields, so we use `ptr::read`
                            // and `mem::forget`.
                            //
                            // NOTE(unsafe) There is no panic branch between getting the resources
                            // and forgetting `self`.
                            unsafe {
                                let buffer = ptr::read(&self.buffer);
                                let payload = ptr::read(&self.payload);
                                mem::forget(self);
                                (buffer, payload)
                            }
                        }
                    }


                    impl<BUFFER, PAYLOAD> Transfer<W, BUFFER, RxDma<PAYLOAD, $CX>>
                    where
                        RxDma<PAYLOAD, $CX>: TransferPayload,
                    {
                        pub fn peek<T>(&self) -> &[T]
                        where
                            BUFFER: AsRef<[T]>,
                        {
                            let pending = self.payload.channel.get_ndtr() as usize;

                            let slice = self.buffer.as_ref();
                            let capacity = slice.len();

                            &slice[..(capacity - pending)]
                        }
                    }

                    impl<RXBUFFER, TXBUFFER, PAYLOAD, TXC> Transfer<W, (RXBUFFER, TXBUFFER), RxTxDma<PAYLOAD, $CX, TXC>>
                    where
                        RxTxDma<PAYLOAD, $CX, TXC>: TransferPayload,
                    {
                        pub fn peek<T>(&self) -> &[T]
                        where
                            RXBUFFER: AsRef<[T]>,
                        {
                            let pending = self.payload.rxchannel.get_ndtr() as usize;

                            let slice = self.buffer.0.as_ref();
                            let capacity = slice.len();

                            &slice[..(capacity - pending)]
                        }
                    }
                )+

                impl DmaExt for $DMAX {
                    type Channels = Channels;

                    fn split(self) -> Channels {
                        let rcc = unsafe { &(*RCC::ptr()) };
                        $DMAX::enable(rcc);

                        // reset the DMA control registers (stops all on-going transfers)
                        $(
                            unsafe{(*$DMAX::ptr()).$cfgrX.reset();}
                        )+

                        Channels((), $($CX { _0: () }),+)
                    }
                }
            }
        )+
    }
}





dma! {
    DMA1: (dma1, {
        C1: (
            cfgr1, CFGR1,
            cntr1, CNTR1,
            paddr1, PADDR1,
            maddr1, MADDR1,
            intfr,
            intfcr,INTFCR,
            htif1, tcif1,
            chtif1, ctcif1, cgif1
        ),
        C2: (
            cfgr2, CFGR2,
            cntr2, CNTR2,
            paddr2, PADDR2,
            maddr2, MADDR2,
            intfr,
            intfcr,INTFCR,
            htif2, tcif2,
            chtif2, ctcif2, cgif2
        ),
        C3: (
            cfgr3, CFGR3,
            cntr3, CNTR3,
            paddr3, PADDR3,
            maddr3, MADDR3,
            intfr,
            intfcr,INTFCR,
            htif3, tcif3,
            chtif3, ctcif3, cgif3
        ),
        C4: (
            cfgr4, CFGR4,
            cntr4, CNTR4,
            paddr4, PADDR4,
            maddr4, MADDR4,
            intfr,
            intfcr,INTFCR,
            htif4, tcif4,
            chtif4, ctcif4, cgif4
        ),
        C5: (
            cfgr5, CFGR5,
            cntr5, CNTR5,
            paddr5, PADDR5,
            maddr5, MADDR5,
            intfr,
            intfcr,INTFCR,
            htif5, tcif5,
            chtif5, ctcif5, cgif5
        ),
        C6: (
            cfgr6, CFGR6,
            cntr6, CNTR6,
            paddr6, PADDR6,
            maddr6, MADDR6,
            intfr,
            intfcr,INTFCR,
            htif6, tcif6,
            chtif6, ctcif6, cgif6
        ),
        C7: (
            cfgr7, CFGR7,
            cntr7, CNTR7,
            paddr7, PADDR7,
            maddr7, MADDR7,
            intfr,
            intfcr,INTFCR,
            htif7, tcif7,
            chtif7, ctcif7, cgif7
        ),
    }),

    DMA2: (dma2, {
        C1: (
            cfgr1, CFGR1,
            cntr1, CNTR1,
            paddr1, PADDR1,
            maddr1, MADDR1,
            intfr,
            intfcr,INTFCR,
            htif1, tcif1,
            chtif1, ctcif1, cgif1
        ),
        C2: (
            cfgr2, CFGR2,
            cntr2, CNTR2,
            paddr2, PADDR2,
            maddr2, MADDR2,
            intfr,
            intfcr,INTFCR,
            htif2, tcif2,
            chtif2, ctcif2, cgif2
        ),
        C3: (
            cfgr3, CFGR3,
            cntr3, CNTR3,
            paddr3, PADDR3,
            maddr3, MADDR3,
            intfr,
            intfcr,INTFCR,
            htif3, tcif3,
            chtif3, ctcif3, cgif3
        ),
        C4: (
            cfgr4, CFGR4,
            cntr4, CNTR4,
            paddr4, PADDR4,
            maddr4, MADDR4,
            intfr,
            intfcr,INTFCR,
            htif4, tcif4,
            chtif4, ctcif4, cgif4
        ),
        C5: (
            cfgr5, CFGR5,
            cntr5, CNTR5,
            paddr5, PADDR5,
            maddr5, MADDR5,
            intfr,
            intfcr,INTFCR,
            htif5, tcif5,
            chtif5, ctcif5, cgif5
        ),
        C6: (
            cfgr6, CFGR6,
            cntr6, CNTR6,
            paddr6, PADDR6,
            maddr6, MADDR6,
            intfr,
            intfcr,INTFCR,
            htif6, tcif6,
            chtif6, ctcif6, cgif6
        ),
        C7: (
            cfgr7, CFGR7,
            cntr7, CNTR7,
            paddr7, PADDR7,
            maddr7, MADDR7,
            intfr,
            intfcr,INTFCR,
            htif7, tcif7,
            chtif7, ctcif7, cgif7
        ),
        C8: (
            cfgr8, CFGR8,
            cntr8, CNTR8,
            paddr8, PADDR8,
            maddr8, MADDR8,
            exten_intfr,
            exten_intfcr,EXTEN_INTFCR,
            htif8, tcif8,
            chtif8, ctcif8, cgif8
        ),
        C9: (
            cfgr9, CFGR9,
            cntr9, CNTR9,
            paddr9, PADDR9,
            maddr9, MADDR9,
            exten_intfr,
            exten_intfcr,EXTEN_INTFCR,
            htif9, tcif9,
            chtif9, ctcif9, cgif9
        ),
        C10: (
            cfgr10, CFGR10,
            cntr10, CNTR10,
            paddr10, PADDR10,
            maddr10, MADDR10,
            exten_intfr,
            exten_intfcr,EXTEN_INTFCR,
            htif10, tcif10,
            chtif10, ctcif10, cgif10
        ),
        C11: (
            cfgr11, CFGR11,
            cntr11, CNTR11,
            paddr11, PADDR11,
            maddr11, MADDR11,
            exten_intfr,
            exten_intfcr,EXTEN_INTFCR,
            htif11, tcif11,
            chtif11, ctcif11, cgif11
        ),
    }),
}

/// DMA Receiver
pub struct RxDma<PAYLOAD, RXCH> {
    pub(crate) payload: PAYLOAD,
    pub channel: RXCH,
}

/// DMA Transmitter
pub struct TxDma<PAYLOAD, TXCH> {
    pub(crate) payload: PAYLOAD,
    pub channel: TXCH,
}

/// DMA Receiver/Transmitter
pub struct RxTxDma<PAYLOAD, RXCH, TXCH> {
    pub(crate) payload: PAYLOAD,
    pub rxchannel: RXCH,
    pub txchannel: TXCH,
}

pub trait Receive {
    type RxChannel;
    type TransmittedWord;
}

pub trait Transmit {
    type TxChannel;
    type ReceivedWord;
}

/// Trait for circular DMA readings from peripheral to memory.
pub trait CircReadDma<B, RS>: Receive
where
    &'static mut [B; 2]: WriteBuffer<Word = RS>,
    B: 'static,
    Self: core::marker::Sized,
{
    fn circ_read(self, buffer: &'static mut [B; 2]) -> CircBuffer<B, Self>;
}

/// Trait for DMA readings from peripheral to memory.
pub trait ReadDma<B, RS>: Receive
where
    B: WriteBuffer<Word = RS>,
    Self: core::marker::Sized + TransferPayload,
{
    fn read(self, buffer: B) -> Transfer<W, B, Self>;
}

/// Trait for DMA writing from memory to peripheral.
pub trait WriteDma<B, TS>: Transmit
where
    B: ReadBuffer<Word = TS>,
    Self: core::marker::Sized + TransferPayload,
{
    fn write(self, buffer: B) -> Transfer<R, B, Self>;
}

/// Trait for DMA simultaneously reading and writing within one synchronous operation. Panics if both buffers are not of equal length.
pub trait ReadWriteDma<RXB, TXB, TS>: Transmit
where
    RXB: WriteBuffer<Word = TS>,
    TXB: ReadBuffer<Word = TS>,
    Self: core::marker::Sized + TransferPayload,
{
    fn read_write(self, rx_buffer: RXB, tx_buffer: TXB) -> Transfer<W, (RXB, TXB), Self>;
}
