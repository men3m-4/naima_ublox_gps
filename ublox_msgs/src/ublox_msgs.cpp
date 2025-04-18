

#include <utility>
#include <vector>

#include <ublox_msgs/serialization.hpp>

template <typename T>
std::vector<std::pair<uint8_t,uint8_t> > ublox::Message<T>::keys_;

DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::NAV, ublox_msgs::Message::NAV::ATT,
                      ublox_msgs, NavATT)
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::NAV, ublox_msgs::Message::NAV::CLOCK,
                      ublox_msgs, NavCLOCK)
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::NAV, ublox_msgs::Message::NAV::DGPS,
                      ublox_msgs, NavDGPS)
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::NAV, ublox_msgs::Message::NAV::DOP,
                      ublox_msgs, NavDOP)
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::NAV, ublox_msgs::Message::NAV::POSECEF,
                      ublox_msgs, NavPOSECEF)
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::NAV, ublox_msgs::Message::NAV::POSLLH,
                      ublox_msgs, NavPOSLLH)
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::NAV,
                      ublox_msgs::Message::NAV::RELPOSNED,
                      ublox_msgs,
                      NavRELPOSNED)
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::NAV,
                      ublox_msgs::Message::NAV::RELPOSNED9,
                      ublox_msgs,
                      NavRELPOSNED9)
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::NAV, ublox_msgs::Message::NAV::SBAS,
                      ublox_msgs, NavSBAS)
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::NAV, ublox_msgs::Message::NAV::SOL,
                      ublox_msgs, NavSOL)
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::NAV, ublox_msgs::Message::NAV::PVT,
                      ublox_msgs, NavPVT)
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::NAV, ublox_msgs::Message::NAV::PVT,
                      ublox_msgs, NavPVT7)
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::NAV, ublox_msgs::Message::NAV::SAT,
                      ublox_msgs, NavSAT)
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::NAV, ublox_msgs::Message::NAV::STATUS,
                      ublox_msgs, NavSTATUS)
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::NAV, ublox_msgs::Message::NAV::SVIN,
                      ublox_msgs, NavSVIN)
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::NAV, ublox_msgs::Message::NAV::SVINFO,
                      ublox_msgs, NavSVINFO)
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::NAV, ublox_msgs::Message::NAV::TIMEGPS,
                      ublox_msgs, NavTIMEGPS)
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::NAV, ublox_msgs::Message::NAV::TIMEUTC,
                      ublox_msgs, NavTIMEUTC)
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::NAV, ublox_msgs::Message::NAV::VELECEF,
                      ublox_msgs, NavVELECEF)
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::NAV, ublox_msgs::Message::NAV::VELNED,
                      ublox_msgs, NavVELNED)

// ACK messages are declared differently because they both have the same
// protocol, so only 1 ROS message is used
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::ACK, ublox_msgs::Message::ACK::NACK,
                      ublox_msgs, Ack)
DECLARE_UBLOX_MESSAGE_ID(ublox_msgs::Class::ACK, ublox_msgs::Message::ACK::ACK,
                      ublox_msgs, Ack, ACK)

// INF messages are declared differently because they all have the same
// protocol, so only 1 ROS message is used. DECLARE_UBLOX_MESSAGE can only
// be called once, and DECLARE_UBLOX_MESSAGE_ID is called for the following
// messages
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::INF, ublox_msgs::Message::INF::ERROR,
                      ublox_msgs, Inf)
DECLARE_UBLOX_MESSAGE_ID(ublox_msgs::Class::INF,
                         ublox_msgs::Message::INF::WARNING,
                         ublox_msgs, Inf, WARNING)
DECLARE_UBLOX_MESSAGE_ID(ublox_msgs::Class::INF,
                         ublox_msgs::Message::INF::NOTICE,
                         ublox_msgs, Inf, NOTICE)
DECLARE_UBLOX_MESSAGE_ID(ublox_msgs::Class::INF,
                         ublox_msgs::Message::INF::TEST,
                         ublox_msgs, Inf, TEST)
DECLARE_UBLOX_MESSAGE_ID(ublox_msgs::Class::INF,
                         ublox_msgs::Message::INF::DEBUG,
                         ublox_msgs, Inf, DEBUG)

DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::RXM, ublox_msgs::Message::RXM::ALM,
                      ublox_msgs, RxmALM)
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::RXM, ublox_msgs::Message::RXM::EPH,
                      ublox_msgs, RxmEPH)
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::RXM, ublox_msgs::Message::RXM::RAW,
                      ublox_msgs, RxmRAW)
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::RXM, ublox_msgs::Message::RXM::RAWX,
                      ublox_msgs, RxmRAWX)
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::RXM, ublox_msgs::Message::RXM::RTCM,
                      ublox_msgs, RxmRTCM)
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::RXM, ublox_msgs::Message::RXM::SFRB,
                      ublox_msgs, RxmSFRB)
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::RXM, ublox_msgs::Message::RXM::SFRBX,
                      ublox_msgs, RxmSFRBX)
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::RXM, ublox_msgs::Message::RXM::SVSI,
                      ublox_msgs, RxmSVSI)

DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::CFG, ublox_msgs::Message::CFG::ANT,
                      ublox_msgs, CfgANT)
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::CFG, ublox_msgs::Message::CFG::CFG,
                      ublox_msgs, CfgCFG)
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::CFG, ublox_msgs::Message::CFG::DAT,
                      ublox_msgs, CfgDAT)
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::CFG, ublox_msgs::Message::CFG::DGNSS,
                      ublox_msgs, CfgDGNSS)
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::CFG, ublox_msgs::Message::CFG::GNSS,
                      ublox_msgs, CfgGNSS)
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::CFG, ublox_msgs::Message::CFG::HNR,
                      ublox_msgs, CfgHNR)
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::CFG, ublox_msgs::Message::CFG::INF,
                      ublox_msgs, CfgINF)
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::CFG, ublox_msgs::Message::CFG::MSG,
                      ublox_msgs, CfgMSG)
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::CFG, ublox_msgs::Message::CFG::NAV5,
                      ublox_msgs, CfgNAV5)
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::CFG, ublox_msgs::Message::CFG::NAVX5,
                      ublox_msgs, CfgNAVX5)
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::CFG, ublox_msgs::Message::CFG::NMEA,
                      ublox_msgs, CfgNMEA)
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::CFG, ublox_msgs::Message::CFG::NMEA,
                      ublox_msgs, CfgNMEA6)
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::CFG, ublox_msgs::Message::CFG::NMEA,
                      ublox_msgs, CfgNMEA7)
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::CFG, ublox_msgs::Message::CFG::PRT,
                      ublox_msgs, CfgPRT)
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::CFG, ublox_msgs::Message::CFG::RATE,
                      ublox_msgs, CfgRATE)
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::CFG, ublox_msgs::Message::CFG::RST,
                      ublox_msgs, CfgRST)
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::CFG, ublox_msgs::Message::CFG::TMODE3,
                      ublox_msgs, CfgTMODE3)
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::CFG, ublox_msgs::Message::CFG::USB,
                      ublox_msgs, CfgUSB)

DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::UPD, ublox_msgs::Message::UPD::SOS,
                      ublox_msgs, UpdSOS)
// SOS and SOS_Ack have the same message ID, but different lengths
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::UPD, ublox_msgs::Message::UPD::SOS,
                      ublox_msgs, UpdSOSAck)

DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::MON, ublox_msgs::Message::MON::GNSS,
                      ublox_msgs, MonGNSS)
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::MON, ublox_msgs::Message::MON::HW,
                      ublox_msgs, MonHW)
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::MON, ublox_msgs::Message::MON::HW,
                      ublox_msgs, MonHW6)
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::MON, ublox_msgs::Message::MON::VER,
                      ublox_msgs, MonVER)

DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::AID, ublox_msgs::Message::AID::ALM,
                      ublox_msgs, AidALM)
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::AID, ublox_msgs::Message::AID::EPH,
                      ublox_msgs, AidEPH)
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::AID, ublox_msgs::Message::AID::HUI,
                      ublox_msgs, AidHUI)

DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::ESF, ublox_msgs::Message::ESF::INS,
                      ublox_msgs, EsfINS)
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::ESF, ublox_msgs::Message::ESF::MEAS,
                      ublox_msgs, EsfMEAS)
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::ESF, ublox_msgs::Message::ESF::RAW,
                      ublox_msgs, EsfRAW)
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::ESF, ublox_msgs::Message::ESF::STATUS,
                      ublox_msgs, EsfSTATUS)


DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::MGA, ublox_msgs::Message::MGA::GAL,
                      ublox_msgs, MgaGAL)

DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::HNR, ublox_msgs::Message::HNR::PVT,
                      ublox_msgs, HnrPVT)

// TIM messages
DECLARE_UBLOX_MESSAGE(ublox_msgs::Class::TIM, ublox_msgs::Message::TIM::TM2,
		      ublox_msgs, TimTM2)

