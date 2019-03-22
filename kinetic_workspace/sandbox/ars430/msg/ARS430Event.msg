Header header

uint8 FAR0_EVENT=0
uint8 FAR1_EVENT=1
uint8 NEAR0_EVENT=2
uint8 NEAR1_EVENT=3
uint8 NEAR2_EVENT=4

string sourceIP
uint8 EventType
uint16 CRC
uint16 Len
uint8 SQC
uint8 MessageCounter
uint64 UtcTimeStamp
uint32 TimeStamp
uint32 MeasureCounter
uint32 CycleCounter
uint16 NofDet
float32 Vambig
uint8 CenterFreq
uint8 DetInPack
RadarDetection[] DetectionList
