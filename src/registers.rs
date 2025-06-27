use num_enum::{IntoPrimitive, TryFromPrimitive};

macro_rules! impl_to_from_u8 {
    ($ident:ident) => {
        impl From<$ident> for u8 {
            fn from(reg: $ident) -> Self {
                reg.0
            }
        }

        impl From<u8> for $ident {
            fn from(int: u8) -> Self {
                Self(int)
            }
        }
    };
}

pub(crate) trait RegisterDefinition: From<u8> + Into<u8> + PartialEq + core::fmt::Debug + Copy {
    fn register() -> ConfigurationRegister;
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum ConfigurationRegister {
    Reg0 = 0,
    Reg1 = 1,
    Reg2 = 2,
    Reg3 = 3,
}

bitfield::bitfield! {
    #[derive(Clone, Copy, PartialEq)]
    pub(crate) struct ConfigurationRegister0(u8);
    impl Debug;

    _mux, _set_mux: 7, 4;
    _gain, _set_gain : 3, 1;
    pub pga_bypass, set_pga_bypass : 0;
}

impl_to_from_u8!(ConfigurationRegister0);

impl RegisterDefinition for ConfigurationRegister0 {
    fn register() -> ConfigurationRegister {
        ConfigurationRegister::Reg0
    }
}

#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord, IntoPrimitive, TryFromPrimitive)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum InputVoltageSource {
    Ain0Ain1 = 0,
    Ain0Ain2 = 1,
    Ain0Ain3 = 2,
    Ain1Ain2 = 3,
    Ain1Ain3 = 4,
    Ain2Ain3 = 5,
    Ain1Ain0 = 6,
    Ain3Ain2 = 7,
    Ain0Avss = 8,
    Ain1Avss = 9,
    Ain2Avss = 10,
    Ain3Avss = 11,
    VrefMonitor = 12,
    AvinMonitor = 13,
    AvddPlusAvss = 14,
}

#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord, IntoPrimitive, TryFromPrimitive)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Gain {
    X1 = 0,
    X2 = 1,
    X4 = 2,
    X8 = 3,
    X16 = 4,
    X32 = 5,
    X64 = 6,
    X128 = 7,
}

impl ConfigurationRegister0 {
    pub fn mux(&self) -> InputVoltageSource {
        InputVoltageSource::try_from(self._mux()).expect("got reserved value for field")
    }

    pub fn set_mux(&mut self, value: InputVoltageSource) {
        self._set_mux(value.into())
    }

    pub fn gain(&self) -> Gain {
        Gain::try_from(self._gain()).expect("all possible values for this field are decodable")
    }

    pub fn set_gain(&mut self, value: Gain) {
        self._set_gain(value.into())
    }
}

bitfield::bitfield! {
    #[derive(Clone, Copy, PartialEq)]
    pub(crate) struct ConfigurationRegister1(u8);
    impl Debug;

    _dr, _set_dr: 7, 5;
    _mode, _set_mode : 4, 3;
    _cm, _set_cm : 2;
    pub ts, set_ts : 1;
    pub bcs, set_bcs : 0;
}

impl_to_from_u8!(ConfigurationRegister1);

impl RegisterDefinition for ConfigurationRegister1 {
    fn register() -> ConfigurationRegister {
        ConfigurationRegister::Reg1
    }
}

#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord, IntoPrimitive, TryFromPrimitive)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum DataRate {
    Normal20 = 0,
    Normal45 = 1,
    Normal90 = 2,
    Normal175 = 3,
    Normal330 = 4,
    Normal600 = 5,
    Normal1000 = 6,
}

#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord, IntoPrimitive, TryFromPrimitive)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum OperatingMode {
    Normal = 0,
    DutyCycle = 1,
    Turbo = 2,
}

#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord, IntoPrimitive, TryFromPrimitive)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum ConversionMode {
    SingleShot = 0,
    Continuous = 1,
}

impl ConfigurationRegister1 {
    pub fn dr(&self) -> DataRate {
        DataRate::try_from(self._dr()).expect("got reserved value for field")
    }

    pub fn set_dr(&mut self, value: DataRate) {
        self._set_dr(value.into())
    }

    pub fn mode(&self) -> OperatingMode {
        OperatingMode::try_from(self._mode()).expect("got reserved value for field")
    }

    pub fn set_mode(&mut self, value: OperatingMode) {
        self._set_mode(value.into())
    }

    pub fn cm(&self) -> ConversionMode {
        ConversionMode::try_from(self._cm() as u8)
            .expect("all possible values for this field are decodable")
    }

    pub fn set_cm(&mut self, value: ConversionMode) {
        self._set_cm(value as u8 != 0)
    }
}

bitfield::bitfield! {
    #[derive(Clone, Copy, PartialEq)]
    pub(crate) struct ConfigurationRegister2(u8);
    impl Debug;

    _vref, _set_vref: 7, 6;
    _fir, _set_fir : 5, 4; // field name is actually "50/60"
    _psw, _set_psw : 3;
    _idac, _set_idac : 2, 0;
}

impl_to_from_u8!(ConfigurationRegister2);

impl RegisterDefinition for ConfigurationRegister2 {
    fn register() -> ConfigurationRegister {
        ConfigurationRegister::Reg2
    }
}

#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord, IntoPrimitive, TryFromPrimitive)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum ReferenceVoltageSource {
    Internal = 0,
    ExternalRef0 = 1,
    ExternalRef1 = 2,
    AnalogSupply = 3,
}

#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord, IntoPrimitive, TryFromPrimitive)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum FirFilterMode {
    None = 0,
    Simultaneous50Hz60Hz = 1,
    Only50Hz = 2,
    Only60Hz = 3,
}

#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord, IntoPrimitive, TryFromPrimitive)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum LowSidePowerSwitchMode {
    AlwaysOpen = 0,
    Automatic = 1,
}

#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord, IntoPrimitive, TryFromPrimitive)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum IdacCurrent {
    Off = 0,
    MicroAmps10 = 1,
    MicroAmps50 = 2,
    MicroAmps100 = 3,
    MicroAmps250 = 4,
    MicroAmps500 = 5,
    MicroAmps1000 = 6,
    MicroAmps1500 = 7,
}

impl ConfigurationRegister2 {
    pub fn vref(&self) -> ReferenceVoltageSource {
        ReferenceVoltageSource::try_from(self._vref())
            .expect("all possible values for this field are decodable")
    }

    pub fn set_vref(&mut self, value: ReferenceVoltageSource) {
        self._set_vref(value.into())
    }

    pub fn fir(&self) -> FirFilterMode {
        FirFilterMode::try_from(self._fir())
            .expect("all possible values for this field are decodable")
    }

    pub fn set_fir(&mut self, value: FirFilterMode) {
        self._set_fir(value.into())
    }

    pub fn psw(&self) -> LowSidePowerSwitchMode {
        LowSidePowerSwitchMode::try_from(self._psw() as u8)
            .expect("all possible values for this field are decodable")
    }

    pub fn set_psw(&mut self, value: LowSidePowerSwitchMode) {
        self._set_psw(value as u8 != 0)
    }

    pub fn idac(&self) -> IdacCurrent {
        IdacCurrent::try_from(self._idac())
            .expect("all possible values for this field are decodable")
    }

    pub fn set_idac(&mut self, value: IdacCurrent) {
        self._set_idac(value.into())
    }
}

bitfield::bitfield! {
      #[derive(Clone, Copy, PartialEq)]
    pub(crate) struct ConfigurationRegister3(u8);
    impl Debug;

    _i1mux, _set_i1mux: 7, 5;
    _i2mux, _set_i2mux : 4, 2;
    _drdym, _set_drdym : 1;
}

impl_to_from_u8!(ConfigurationRegister3);

impl RegisterDefinition for ConfigurationRegister3 {
    fn register() -> ConfigurationRegister {
        ConfigurationRegister::Reg3
    }
}

#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord, IntoPrimitive, TryFromPrimitive)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum IdacRoutingDestination {
    Disabled = 0,
    Ain0Refp1 = 1,
    Ain1 = 2,
    Ain2 = 3,
    Ain3Refn1 = 4,
    Refp0 = 5,
    Refn0 = 6,
}

#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord, IntoPrimitive, TryFromPrimitive)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum DrdyMode {
    OnlyDrdy,
    DoutAndDrdy,
}

impl ConfigurationRegister3 {
    pub fn i1mux(&self) -> IdacRoutingDestination {
        IdacRoutingDestination::try_from(self._i1mux()).expect("got reserved value for field")
    }

    pub fn set_i1mux(&mut self, value: IdacRoutingDestination) {
        self._set_i1mux(value.into())
    }

    pub fn i2mux(&self) -> IdacRoutingDestination {
        IdacRoutingDestination::try_from(self._i2mux()).expect("got reserved value for field")
    }

    pub fn set_i2mux(&mut self, value: IdacRoutingDestination) {
        self._set_i2mux(value.into())
    }

    pub fn drdym(&self) -> DrdyMode {
        DrdyMode::try_from(self._drdym() as u8)
            .expect("all possible values for this field are decodable")
    }

    pub fn set_drdym(&mut self, value: DrdyMode) {
        self._set_drdym(value as u8 != 0)
    }
}
