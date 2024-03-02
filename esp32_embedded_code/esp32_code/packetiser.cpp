#include "packetiser.hpp"

namespace Packetiser{

	bool Packetiser::matchEsc(std::vector<uint8_t>::iterator seq){
		std::pair<uint8_t, uint8_t> check = {*seq, *(seq + 1)};
		bool result = false;
		for(auto it = ESC_SEQUENCES.begin(); it != ESC_SEQUENCES.end(); ++it){
			if(check == it->second)
				result = true;
		}
		return result;
	}

	Buffer Packetiser::encapsulatePacket(Buffer &buff){
		Buffer output = {(uint8_t)START_MARKER};

		// add packet while escaping special bytes
		for(int i = 0; i < buff.size(); i++){
			uint8_t byte = buff[i];
			bool escaped = false;

			for(auto it = ESC_SEQUENCES.begin(); it != ESC_SEQUENCES.end(); ++it){
				if(byte == it->first){
					output.push_back(it->second.first);
					output.push_back(it->second.second);
					escaped = true;
					break;
				}
			}

			if(!escaped)
				output.push_back(byte);
		}

		output.push_back(STOP_MARKER);

		return output;
	}

	void Packetiser::processInput(Buffer &data, std::vector<std::pair<Buffer,PacketType>> &packetQueue, bool returnGarbage, bool debug){
		Buffer outputs = {};
		bool partialPacketFound = false;
		
		// combine previous input buffer and new data
		stream.insert(stream.end(),data.begin(),data.end());

		int maxLoop = 1000;

		// while a partial packet has not been found and there is data in the buffer continue to process the buffer
		while(!partialPacketFound && stream.size() > 0 && maxLoop > 0){

			maxLoop--;

			// if the first byte is not a start byte, then create a garbage segment of discarded bytes until a
			// start byte is found
			if(stream[0] != START_MARKER){
				Buffer garbageBytes = {};
				while(stream.size() > 0 && stream[0] != START_MARKER){
				garbageBytes.push_back(stream[0]);
				stream.erase(stream.begin());
				}
				if(returnGarbage){
					packetQueue.push_back(std::make_pair(garbageBytes,PacketType::garbage));
				}
			}

			// if there are any bytes left then they must start with a start marker so start to identify packet
			auto streamPos = stream.begin() + 1;
			Buffer decapsulatedPacket = {};
			bool packetComplete = false;
			bool garbagePacketFound = false;
			while(stream.size() > (streamPos - stream.begin())){

				// if a stop marker is found then complete the packet
				if(*streamPos == STOP_MARKER){
					streamPos++;
					packetComplete = true;
					break;
				}
				// if a start marker was found (should not happen unless some garbage data happened to include a start marker)
				// then create a garbage section
				else if(*streamPos == START_MARKER){
					Buffer garbageSection(stream.begin(), streamPos);
					packetQueue.push_back(std::make_pair(garbageSection,PacketType::garbage));
					stream.erase(stream.begin(), streamPos);
					garbagePacketFound = true;
					break;
				}
				// if an escape sequence was found then decode it
				else if(matchEsc(streamPos)){
					std::pair<uint8_t,uint8_t> p(*streamPos, *(streamPos+1));
					for(auto it = ESC_SEQUENCES.begin(); it != ESC_SEQUENCES.end(); ++it){
						if(p == it->second){
							decapsulatedPacket.push_back(it->first);
							streamPos += 2;
							break;
						}
					}
				}
				// if the byte was not a start/stop marker or escape sequence then add verbatim
				else{
					decapsulatedPacket.push_back(*streamPos);
					streamPos++;
				}
			}

			// if a complete packet was found then add it to the output and remove the encapsulated
			// version from the input stream
			if(packetComplete){
				packetQueue.push_back(std::make_pair(decapsulatedPacket, PacketType::packet));
				stream.erase(stream.begin(), streamPos);
			}
			// if the packet was not complete then flag an incomplete packet on the input buffer
			else if(streamPos == stream.end()){
				partialPacketFound = true;
			}
			
		}
	}
}
