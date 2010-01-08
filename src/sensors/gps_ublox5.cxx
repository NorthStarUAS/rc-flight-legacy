/**
 * \file: gps_ublox5.cpp
 *
 * u-blox 5 protocol driver
 *
 * Copyright Curt Olson curtolson@gmail.com
 *
 * $Id: gpsd.cpp,v 1.7 2009/08/25 15:04:01 curt Exp $
 */

#include <math.h>		// sin() cos()
#include <sys/types.h>		// open()
#include <sys/stat.h>		// open()
#include <fcntl.h>		// open()
#include <stdio.h>		// printf() et. al.
#include <termios.h>		// tcgetattr() et. al.
#include <unistd.h>		// tcgetattr() et. al.
#include <string.h>		// memset()
#include <sys/time.h>		// gettimeofday()
#include <string>

using std::string;

#include "globaldefs.h"

#include "comms/logging.h"
#include "props/props.hxx"
#include <util/strutils.hxx>
#include "util/timing.h"
#include "gps_mgr.h"

#include "gps_ublox5.h"


#define UBX_SYNC1 0xB5
#define UBX_SYNC2 0x62

typedef enum { 	
	NO_STATE,
	GOT_SYNC1,
	GOT_SYNC2,
	GOT_CLASS,
	GOT_ID,
	GOT_LENGTH1,
	GOT_LENGTH2,
	GOT_PAYLOAD,
	GOT_CHECKSUM1,
	GOT_CHECKSUM2,
	GOT_UBX_PACKET,
} UBX_PARSE_STATE;

#pragma pack(push, 1)
typedef struct {
    uint8_t sync1;
    uint8_t sync2;
    uint8_t class_var;
    uint8_t id;
    uint16_t length;
    uint8_t payload[255];
} ubxGenericPkt_t;
#pragma pack(pop)

static ubxGenericPkt_t ubxPkt;
static UBX_PARSE_STATE state = NO_STATE;
static struct timeval timestamp; // Unix Time Stamp Structure

// gpsd property nodes
static SGPropertyNode *configroot = NULL;

static SGPropertyNode *gps_timestamp_node = NULL;
static SGPropertyNode *gps_lat_node = NULL;
static SGPropertyNode *gps_lon_node = NULL;
static SGPropertyNode *gps_alt_node = NULL;
static SGPropertyNode *gps_ve_node = NULL;
static SGPropertyNode *gps_vn_node = NULL;
static SGPropertyNode *gps_vd_node = NULL;
static SGPropertyNode *gps_unix_sec_node = NULL;

static SGPropertyNode *gps_satellites = NULL;
static SGPropertyNode *gps_nmode = NULL;

static SGPropertyNode *gps_device_name_node = NULL;

static int fd = -1;
static string device_name = "/dev/ttyS0";


// initialize gpsd input property nodes
static void bind_input( SGPropertyNode *config ) {
    gps_device_name_node = config->getChild("device");
    if ( gps_device_name_node != NULL ) {
	device_name = gps_device_name_node->getStringValue();
    }
    configroot = config;
}


// initialize gpsd output property nodes 
static void bind_output( string rootname ) {
    SGPropertyNode *outputroot = fgGetNode( rootname.c_str(), true );
    gps_timestamp_node = outputroot->getChild("time-stamp", 0, true);
    gps_lat_node = outputroot->getChild("latitude-deg", 0, true);
    gps_lon_node = outputroot->getChild("longitude-deg", 0, true);
    gps_alt_node = outputroot->getChild("altitude-m", 0, true);
    gps_ve_node = outputroot->getChild("ve-ms", 0, true);
    gps_vn_node = outputroot->getChild("vn-ms", 0, true);
    gps_vd_node = outputroot->getChild("vd-ms", 0, true);
    gps_unix_sec_node = outputroot->getChild("unix-time-sec", 0, true);

    gps_satellites = outputroot->getChild("satellites", 0, true);
    gps_nmode = outputroot->getChild("nmea-mode", 0, true);
}


// send our configured init strings to configure gpsd the way we prefer
static bool gps_ublox5_open() {
    fd = open( device_name.c_str(), O_RDONLY | O_NOCTTY );
    if ( fd < 0 ) {
	printf("Error opening device: %s", device_name.c_str());
	perror("");
	printf("\n");
	return false;
    }

    struct termios oldTio;	// Old Serial Port Settings
    struct termios newTio; 	// New Serial Port Settings
    memset(&oldTio, 0, sizeof(oldTio));
    memset(&newTio, 0, sizeof(newTio));

    // Save Current Serial Port Settings
    tcgetattr(fd,&oldTio); 

    // Configure New Serial Port Settings
    newTio.c_cflag     = B115200 | // bps rate
                         CRTSCTS | // output flow ctnl
                         CS8	 | // 8n1
                         CLOCAL	 | // local connection, no modem
                         CREAD;	   // enable receiving chars
    newTio.c_iflag     = IGNPAR;   // ignore parity bits
    newTio.c_oflag     = 0;
    newTio.c_lflag     = 0;
    newTio.c_cc[VTIME] = 0;
    newTio.c_cc[VMIN]  = 1;	   // block 'read' from returning until at
                                   // least 1 character is received

    // Flush Serial Port I/O buffer
    tcflush(fd, TCIOFLUSH);

    // Set New Serial Port Settings
    int ret = tcsetattr( fd, TCSANOW, &newTio );
    if ( ret > 0 ) {
	printf("Error configuring device: %s", device_name.c_str());
	perror("");
	printf("\n");
	return false;
    }

    return true;
}


void gps_ublox5_init( string rootname, SGPropertyNode *config ) {
    bind_input( config );
    bind_output( rootname );
    gps_ublox5_open();
}


static bool parse_ublox5_sentence( const char *sentence ) {
    bool new_position = false;

    uint8_t * pktPtr = (uint8_t *) &ubxPkt;
    uint8_t * pktIndex = pktPtr;
    uint8_t payloadIndex =0;
    uint8_t ck_a = 0;
    uint8_t ck_b = 0;

    // Read 1 byte
    read(fd, pktIndex, 1);

    if (state < GOT_PAYLOAD) {
	ck_a += *pktIndex;
	ck_b += ck_a;
    }

    switch (state) {

    case NO_STATE:
	// If byte read is 1st sync byte update state
	if (ubxPkt.sync1  == UBX_SYNC1) {
	    state = GOT_SYNC1;	
	    pktIndex++;
	} else {
	    state = NO_STATE;
	    pktIndex = pktPtr;
	    memset(pktPtr, 0, 1);
	}
	break;

    case GOT_SYNC1:
	// If byte read is 1st sync byte update state
	if (ubxPkt.sync2  == UBX_SYNC2) {
	    state = GOT_SYNC2;	
	    pktIndex++;
	    ck_a = 0;
	    ck_b = 0;
	} else {
	    state = NO_STATE;
	    pktIndex = pktPtr;
	    memset(pktPtr, 0, 2);
	}	
	break;

    case GOT_SYNC2:
	state = GOT_CLASS;	
	pktIndex++;
	break;

    case GOT_CLASS:
	state = GOT_ID;
	pktIndex++;	
	break;

    case GOT_ID:
	state = GOT_LENGTH1;
	pktIndex++;
	break;
				
    case GOT_LENGTH1:
	state = GOT_LENGTH2;
	pktIndex++;
	payloadIndex = 0;
	//printf("Sync 1 %x Sync 2 %x Class %x Id %x Length %x\n", ubxPkt.sync1, ubxPkt.sync2, ubxPkt.class, ubxPkt.id, ubxPkt.length);
	break;

    case GOT_LENGTH2:
	payloadIndex++; 
	if (payloadIndex == ubxPkt.length) {
	    state = GOT_PAYLOAD;
	}
	pktIndex++;
	break;
			
    case GOT_PAYLOAD:
	//printf("%x \n", pktIndex);
	if (ck_a == *pktIndex) {
	    state = GOT_CHECKSUM1;
	    pktIndex++;		
	} else {
	    printf("CK A failed: act %x cal %x\n", *pktIndex, ck_a);
	    state = NO_STATE;
	    pktIndex = pktPtr;
	    memset(pktPtr, 0, sizeof(ubxGenericPkt_t));
	}	
	break;

    case GOT_CHECKSUM1:
	if (ck_b == *pktIndex) { 
	    state = GOT_UBX_PACKET; 
	} else {
	    printf("CK B failed: act %x cal %x\n", *pktIndex, ck_b);
	    state = NO_STATE;
	    memset(pktPtr, 0, sizeof(ubxGenericPkt_t));
	    pktIndex = pktPtr;
	}
	break;

    case GOT_UBX_PACKET:
	gettimeofday(&timestamp, NULL);
	printf("time %f\n", timestamp.tv_sec + timestamp.tv_usec / 1000000.);
	uint32_t iTOW = (ubxPkt.payload[3] <<24)+ (ubxPkt.payload[2] <<16) + (ubxPkt.payload[1] <<8) + ubxPkt.payload[0];
	printf("ITOW = %d", iTOW);
	printf("Sync 1 %x Sync 2 %x Class %x Id %x Length %x\n", ubxPkt.sync1, ubxPkt.sync2, ubxPkt.class_var, ubxPkt.id, ubxPkt.length);

	// DO SOME PARSING HERE
	state = NO_STATE; memset(pktPtr, 0, sizeof(ubxGenericPkt_t));	
	break;
    }
    if (pktIndex > (pktPtr + 255)) {
	pktIndex = pktPtr;
    }

    return new_position;
}


bool gps_ublox5_update() {
    bool gps_data_valid = false;

    // run an iteration of the ublox5 parser

    return gps_data_valid;
 }


void gps_ublox5_close() {
}
