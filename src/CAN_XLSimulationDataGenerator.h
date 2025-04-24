#ifndef CAN_XL_SIMULATION_DATA_GENERATOR
#define CAN_XL_SIMULATION_DATA_GENERATOR

#include <SimulationChannelDescriptor.h>
#include <string>
class CAN_XLAnalyzerSettings;

class CAN_XLSimulationDataGenerator
{
public:
	CAN_XLSimulationDataGenerator();
	~CAN_XLSimulationDataGenerator();

	void Initialize( U32 simulation_sample_rate, CAN_XLAnalyzerSettings* settings );
	U32 GenerateSimulationData( U64 newest_sample_requested, U32 sample_rate, SimulationChannelDescriptor** simulation_channel );

protected:
	CAN_XLAnalyzerSettings* mSettings;
	U32 mSimulationSampleRateHz;

protected:
	void CreateSerialByte();
	std::string mSerialText;
	U32 mStringIndex;

	SimulationChannelDescriptor mSerialSimulationData;

};
#endif //CAN_XL_SIMULATION_DATA_GENERATOR