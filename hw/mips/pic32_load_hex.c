#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "pic32_peripherals.h"

/* Macros for converting between hex and binary. */
#define NIBBLE(x)       (isdigit(x) ? (x)-'0' : tolower(x)+10-'a')
#define HEX(buffer)     ((NIBBLE((buffer)[0])<<4) + NIBBLE((buffer)[1]))

static unsigned virt_to_phys(unsigned address)
{
    if (address >= 0xa0000000 && address <= 0xbfffffff)
        return address - 0xa0000000;
    if (address >= 0x80000000 && address <= 0x9fffffff)
        return address - 0x80000000;
    return address;
}

/*
 * Read the S record file.
 */
static int load_srec(const char *filename,
    void (*store_byte)(unsigned address, unsigned char byte))
{
    FILE *fd;
    unsigned char buf [256];
    unsigned char *data;
    unsigned address;
    int bytes, output_len;

    fd = fopen(filename, "r");
    if (! fd) {
        perror(filename);
        exit(1);
    }
    output_len = 0;
    while (fgets((char*) buf, sizeof(buf), fd)) {
        if (buf[0] == '\n')
            continue;
        if (buf[0] != 'S') {
            if (output_len == 0)
                break;
            printf("%s: bad file format\n", filename);
            exit(1);
        }

        /* Starting an S-record.  */
        if (! isxdigit(buf[2]) || ! isxdigit(buf[3])) {
            printf("%s: bad record: %s\n", filename, buf);
            exit(1);
        }
        bytes = HEX(buf + 2);

        /* Ignore the checksum byte.  */
        --bytes;

        address = 0;
        data = buf + 4;
        switch (buf[1]) {
        case '7':
            address = HEX(data);
            data += 2;
            --bytes;
            /* Fall through.  */
        case '8':
            address = (address << 8) | HEX(data);
            data += 2;
            --bytes;
            /* Fall through.  */
        case '9':
            address = (address << 8) | HEX(data);
            data += 2;
            address = (address << 8) | HEX(data);
            data += 2;
            bytes -= 2;
            if (bytes == 0) {
                //printf("%s: start address = %08x\n", filename, address);
                //TODO: set start address
            }
            goto done;

        case '3':
            address = HEX(data);
            data += 2;
            --bytes;
            /* Fall through.  */
        case '2':
            address = (address << 8) | HEX(data);
            data += 2;
            --bytes;
            /* Fall through.  */
        case '1':
            address = (address << 8) | HEX(data);
            data += 2;
            address = (address << 8) | HEX(data);
            data += 2;
            bytes -= 2;

            address = virt_to_phys(address);
            output_len += bytes;
            while (bytes-- > 0) {
                store_byte(address++, HEX(data));
                data += 2;
            }
            break;
        }
    }
done:
    fclose(fd);
    return output_len;
}

/*
 * Read HEX file.
 */
static int load_hex(const char *filename,
    void (*store_byte)(unsigned address, unsigned char byte))
{
    FILE *fd;
    unsigned char buf [256], data[16], record_type, sum;
    unsigned address, high;
    int bytes, output_len, i;

    fd = fopen(filename, "r");
    if (! fd) {
        perror(filename);
        exit(1);
    }
    output_len = 0;
    high = 0;
    while (fgets((char*) buf, sizeof(buf), fd)) {
        if (buf[0] == '\n')
            continue;
        if (buf[0] != ':') {
            if (output_len == 0)
                break;
            printf("%s: bad HEX file format\n", filename);
            exit(1);
        }
        if (! isxdigit(buf[1]) || ! isxdigit(buf[2]) ||
            ! isxdigit(buf[3]) || ! isxdigit(buf[4]) ||
            ! isxdigit(buf[5]) || ! isxdigit(buf[6]) ||
            ! isxdigit(buf[7]) || ! isxdigit(buf[8])) {
            printf("%s: bad record: %s\n", filename, buf);
            exit(1);
        }
        record_type = HEX(buf+7);
        if (record_type == 1) {
            /* End of file. */
            break;
        }
        bytes = HEX(buf+1);
        if (strlen((char*) buf) < bytes * 2 + 11) {
            printf("%s: too short hex line\n", filename);
            exit(1);
        }
        address = high << 16 | HEX(buf+3) << 8 | HEX(buf+5);
        if (address & 3) {
            printf("%s: odd address\n", filename);
            exit(1);
        }

        sum = 0;
        for (i=0; i<bytes; ++i) {
            data [i] = HEX(buf+9 + i + i);
            sum += data [i];
        }
        sum += record_type + bytes + (address & 0xff) + (address >> 8 & 0xff);
        if (sum != (unsigned char) - HEX(buf+9 + bytes + bytes)) {
            printf("%s: bad hex checksum\n", filename);
            printf("Line %s", buf);
            exit(1);
        }

        if (record_type == 5) {
            /* Start address. */
            if (bytes != 4) {
                printf("%s: invalid length of hex start address record: %d bytes\n",
                    filename, bytes);
                exit(1);
            }
            address = data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3];
            //printf("%s: start address = %08x\n", filename, address);
            //TODO: set start address
            continue;
        }
        if (record_type == 4) {
            /* Extended address. */
            if (bytes != 2) {
                printf("%s: invalid length of hex linear address record: %d bytes\n",
                    filename, bytes);
                exit(1);
            }
            high = data[0] << 8 | data[1];
            continue;
        }
        if (record_type != 0) {
            printf("%s: unknown hex record type: %d\n",
                filename, record_type);
            exit(1);
        }

        /* Data record found. */
        address = virt_to_phys(address);
        output_len += bytes;
        for (i=0; i<bytes; i++) {
            store_byte(address++, data [i]);
        }
    }
    fclose(fd);
    return output_len;
}

int pic32_load_hex_file(const char *filename,
    void (*store_byte)(unsigned address, unsigned char byte))
{
    int memory_len = load_srec(filename, store_byte);
    if (memory_len == 0) {
        memory_len = load_hex(filename, store_byte);
        if (memory_len == 0) {
            return 0;
        }
    }
    printf("Load file: '%s', %d bytes\n", filename, memory_len);
    if (qemu_logfile)
        fprintf(qemu_logfile, "Load file: '%s', %d bytes\n", filename, memory_len);
    return 1;
}
