#define CRC16 0x8005


unsigned int gen_crc16(const uint8_t *data, uint16_t size)
{
    uint16_t out = 0;
    int bits_read = 0, bit_flag;

    if(data == NULL)
        return 0;

    while(size > 0)
    {
        bit_flag = out >> 15;

        out <<= 1;
        out |= (*data >> (7 - bits_read)) & 1;

        bits_read++;
        if(bits_read > 7)
        {
            bits_read = 0;
            data++;
            size--;
        }

        if(bit_flag)
            out ^= CRC16;

    }
    return out;
}

