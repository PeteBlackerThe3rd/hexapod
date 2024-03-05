#ifndef _ENCAPSULATION_H_
#define _ENCAPSULATION_H_

#include <map>
#include <vector>
#include <iostream>
#include "buffer.hpp"

namespace Packetiser{
  
	enum PacketType { packet = 0, garbage = 1};
  //typedef std::vector<uint8_t> Buffer;
  typedef std::vector<std::pair<Buffer,PacketType>> PacketQueue;


	class Packetiser{
		public:
			Buffer stream;
			bool matchEsc(std::vector<uint8_t>::iterator seq);
			Buffer encapsulatePacket(Buffer &buff);
			void processInput(Buffer &data, PacketQueue &packetQueue, bool return_garbage, bool debug);

		private:
		// Define Encapsulation codes
		const int START_MARKER = 0xa1;
		const int STOP_MARKER = 0xa2;

		const std::map<uint8_t,std::pair<uint8_t,uint8_t>> ESC_SEQUENCES = {
		{START_MARKER, {0xa3, 0xa4}},
		{STOP_MARKER, {0xa3, 0xa5}},
		{0xa3, {0xa3, 0xa6}}
		};
	};
}
#endif