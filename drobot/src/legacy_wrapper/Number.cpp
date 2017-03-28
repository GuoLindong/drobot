#include "drobot/legacy_wrapper/Number.h"
#include <cstdlib>
#include <ros/ros.h>

using namespace std;

namespace drobot
{
	//int to char
	void utob(void *dest, size_t dest_len, uint64_t src)
	{
		size_t i;
		/* Copy bytes from int to array */
		for(i = 0; (i < dest_len) && (i < sizeof(uint64_t)); ++i)
		{
			((uint8_t *) dest)[i] = (src >> (i * 8)) & 0xff;
		}
		/* If array is larger than int, 0-fill the remainder */
		for (; i < dest_len; ++i)
		{
			((uint8_t *) dest)[i] = 0;
		}
	}

	void itob(void *dest, size_t dest_len, int64_t src)
	{
		size_t i;
		/* Copy bytes from int to array */
		for (i = 0; (i < dest_len) && (i < sizeof(int64_t)); ++i)
		{
			((uint8_t *) dest)[i] = (src >> (i * 8)) & 0xff;
		}
		/* If array is larger than int, sign-fill the remainder */
		for (; i < dest_len; ++i)
		{
			if (((uint8_t *) dest)[dest_len - 1] & 0x80)
			{ // MSB is set, int is negative
				((uint8_t *) dest)[i] = 0xff;
			}
			else
			{ // int is positive
				((uint8_t *) dest)[i] = 0;
			}
		}
	}

	void ftob(void *dest, size_t dest_len, double src, double scale)
	{
		int64_t int_src = (src * scale);
		itob(dest, dest_len, int_src);
	}

	/* Need to provide all these overloaded functions because integer promotion
	* of smaller int types is ambiguous between the uint64/int64 */
	void utob(void *dest, size_t dest_len, uint32_t src)
	{
		utob(dest, dest_len, (uint64_t) src);
	}

	void utob(void *dest, size_t dest_len, uint16_t src)
	{
		utob(dest, dest_len, (uint64_t) src);
	}

	void itob(void *dest, size_t dest_len, int32_t src)
	{
	  itob(dest, dest_len, (int64_t) src);
	}

	void itob(void *dest, size_t dest_len, int16_t src)
	{
	  itob(dest, dest_len, (int64_t) src);
	}

	uint64_t btou(void *src, size_t src_len)
	{
		uint64_t retval = 0;

		if(!src_len) { return 0; }
		size_t i = src_len -1;
		do
		{
			retval = retval << 8;
			retval |= ((uint8_t *) src)[i];
		} while (i--);

		return retval;
	}

	int32_t btoi(void *src, size_t src_len)
	{
		int32_t retval = 0;
		size_t i = sizeof(int32_t);

		if(!src_len) { return 0;}

		for(; i >= src_len; --i)
		{
			retval = retval << 8;
			if (((uint8_t *) src)[src_len - 1] & 0x80)
			{  // MSB is set, int is negative
			  retval |= 0xff;
			}
		}
		do
		{
			
			retval = retval << 8;
			retval |= ((uint8_t *) src)[i];
		} while (i--);
		return retval;
	}

	double btof(void *src, size_t src_len, double scale)
	{
		double retval = btoi(src, src_len);
		return retval /= scale;
	}

}; // namespace drobot