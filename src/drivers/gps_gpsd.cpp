#include "math.h"
#include "rapidjson/document.h"
#include "time.h"

#include "gps_gpsd.h"

// connect to gpsd
void gpsd_t::connect() {
    // make sure it's closed
    gpsd_sock.close();

    if ( display_on ) {
	printf("Attempting to connect to gpsd @ %s:%d ... ",
	       host.c_str(), port);
    }

    if ( ! gpsd_sock.open( true ) ) {
	if ( display_on ) {
	    printf("error opening gpsd socket\n");
	}
	return;
    }
    
    if (gpsd_sock.connect( host.c_str(), port ) < 0) {
	if ( display_on ) {
	    printf("error connecting to gpsd\n");
	}
	return;
    }

    gpsd_sock.setBlocking( false );

    socket_connected = true;

    send_init();

    if ( display_on ) {
	printf("success!\n");
    }
}

// send our configured init strings to configure gpsd the way we prefer
void gpsd_t::send_init() {
    if ( !socket_connected ) {
	return;
    }

    if ( init_string != "" ) {
	if ( display_on ) {
	    printf("sending to gpsd: %s\n", init_string.c_str());
	}
	if ( gpsd_sock.send( init_string.c_str(), init_string.length() ) < 0 ) {
	    socket_connected = false;
	}
    }

    last_init_time = get_Time();
}

void gpsd_t::init( pyPropertyNode *config ) {
    if ( config->hasChild("port") ) {
	port = config->getLong("port");
    }
    if ( config->hasChild("host") ) {
	host = config->getString("host");
    }
    if ( config->hasChild("init_string") ) {
	init_string = config->getString("init_string");
    }
    bool primary = false;
    if ( config->hasChild("primary") ) {
        primary = config->getBool("primary");
    }
    string output_path = get_next_path("/sensors", "gps", primary);
    gps_node = pyGetNode(output_path.c_str(), true);
    raw_node = gps_node.getChild("sats_raw", true);
}

bool gpsd_t::parse_message(const string message) {
    // printf("parse: %s\n", message.c_str());
    rapidjson::Document d;
    d.Parse(message.c_str());
    if ( !d.HasMember("class") ) {
        return false;
    }
    string msg_class = d["class"].GetString();
    if ( msg_class == "VERSION" ) {
        printf("gpsd: %s\n", message.c_str());
    } else if ( msg_class == "TPV" ) {
        // time, pos, vel
        if ( d.HasMember("time") ) {
            string time_str = d["time"].GetString();
            struct tm t;
            char* ptr = strptime(time_str.c_str(), "%FT%T", &t);
            if ( ptr == nullptr ) {
                printf("gpsd: unable to parse time string = %s\n",
                       time_str.c_str());
            } else {
                double t2 = timegm(&t); // UTC
                if ( *ptr ) {
                    double fraction = atof(ptr);
                    t2 += fraction;
                }
                gps_node.setDouble( "unix_time_sec", t2 );
                gps_node.setDouble( "timestamp", get_Time() );
            }
        }
        if ( d.HasMember("lat") ) {
            gps_node.setDouble( "latitude_deg", d["lat"].GetDouble() );
        }
        if ( d.HasMember("lon") ) {
            gps_node.setDouble( "longitude_deg", d["lon"].GetDouble() );
        }
        if ( d.HasMember("alt") ) {
            gps_node.setDouble( "altitude_m", d["alt"].GetFloat() );
        }
        float course_deg = 0.0;
        if ( d.HasMember("track") ) {
            course_deg = d["track"].GetFloat();
        }
        float speed_mps = 0.0;
        if ( d.HasMember("speed") ) {
            speed_mps = d["speed"].GetFloat();
        }
        float angle_rad = (90.0 - course_deg) * M_PI/180.0;
        gps_node.setDouble( "vn_ms", sin(angle_rad) * speed_mps );
        gps_node.setDouble( "ve_ms", cos(angle_rad) * speed_mps );
        if ( d.HasMember("climb") ) {
            gps_node.setDouble( "vd_ms", -d["climb"].GetFloat() );
        }
        if ( d.HasMember("mode") ) {
            gps_node.setLong( "fixType", d["mode"].GetInt() );
        }
    } else if ( msg_class == "SKY" ) {
        if ( d.HasMember("satellites") ) {
            int num_sats = 0;
            const rapidjson::Value& sats = d["satellites"];
            if ( sats.IsArray() ) {
                for (rapidjson::SizeType i = 0; i < sats.Size(); i++) {
                    if ( sats[i].HasMember("used") ) {
                        if ( sats[i]["used"].GetBool() ) {
                            num_sats++;
                        }
                    }
                }
            }
            gps_node.setLong( "satellites", num_sats );
        }
    } else if ( msg_class == "RAW" ) {
        double timestamp = 0.0;
        if ( d.HasMember("time") ) {
            timestamp = d["time"].GetDouble();
            if ( d.HasMember("nsec") ) {
                timestamp += d["nsec"].GetDouble() / 1000000.0;
            }
        }
        if ( d.HasMember("rawdata") ) {
            const rapidjson::Value& raw = d["rawdata"];
            if ( raw.IsArray() ) {
                for (rapidjson::SizeType i = 0; i < raw.Size(); i++) {
                    int gnssid = -1;
                    int svid = -1;
                    double pr = 0.0;
                    double doppler = 0.0;
                    string id_str = "";
                    if ( raw[i].HasMember("gnssid") ) {
                        gnssid = raw[i]["gnssid"].GetInt();
                        id_str = std::to_string(gnssid);
                    }
                    if ( raw[i].HasMember("svid") ) {
                        svid = raw[i]["svid"].GetInt();
                        id_str += "-";
                        id_str += std::to_string(svid);
                    }
                    if ( raw[i].HasMember("pseudorange") ) {
                        pr = raw[i]["pseudorange"].GetDouble();
                    }
                    if ( raw[i].HasMember("doppler") ) {
                        doppler = raw[i]["doppler"].GetDouble();
                    }
                    pyPropertyNode node = raw_node.getChild(id_str.c_str(), true);
                    if ( ! node.hasChild("frame1") ) {
                        node.setBool("frame1", false);
                    }
                    if ( ! node.hasChild("frame2") ) {
                        node.setBool("frame2", false);
                    }
                    if ( ! node.hasChild("frame3") ) {
                        node.setBool("frame3", false);
                    }
                    node.setLong("gnssid", gnssid);
                    if ( gnssid == 0 ) {
                        node.setString("constellation", "gps");
                    } else if ( gnssid == 1 ) {
                        node.setString("constellation", "sbas"); 
                    } else if ( gnssid == 2 ) {
                        node.setString("constellation", "galileo");
                    } else if ( gnssid == 3 ) {
                        node.setString("constellation", "beidou");
                    } else if ( gnssid == 4 ) {
                        node.setString("constellation", "imes");
                    } else if ( gnssid == 5 ) {
                        node.setString("constellation", "qzss");
                    } else if ( gnssid == 6 ) {
                        node.setString("constellation", "glonass");
                    }
                    node.setDouble("pseudorange", pr);
                    node.setDouble("doppler", doppler);
                    node.setDouble("timestamp", timestamp);
                }
            }
        }
    } else if ( msg_class == "SUBFRAME" ) {
        // printf("parse: %s\n", message.c_str());
        int tSV = -1;
        string id_str = "none";
        if ( d.HasMember("tSV") ) {
            tSV = d["tSV"].GetInt();
            id_str = "0-";
            id_str += std::to_string(tSV);
        }
        pyPropertyNode node = raw_node.getChild(id_str.c_str(), true);
        int frame = 0;
        if ( d.HasMember("TOW17") ) {
            node.setLong( "TOW17", d["TOW17"].GetInt() );
        }
        if ( d.HasMember("frame") ) {
            frame = d["frame"].GetInt();
        }
        if ( frame == 1 and d.HasMember("EPHEM1") ) {
            node.setBool("frame1", true);
            const rapidjson::Value& e1 = d["EPHEM1"];
            if ( e1.HasMember("WN") ) {
                node.setLong("WN", e1["WN"].GetInt());
            }
            if ( e1.HasMember("IODC") ) {
                node.setLong("IODC", e1["IODC"].GetInt());
            }
            if ( e1.HasMember("L2") ) {
                node.setLong("L2", e1["L2"].GetInt());
            }
            if ( e1.HasMember("ura") ) {
                node.setLong("ura", e1["ura"].GetInt());
            }
            if ( e1.HasMember("hlth") ) {
                node.setLong("hlth", e1["hlth"].GetInt());
            }
            if ( e1.HasMember("L2P") ) {
                node.setLong("L2P", e1["L2P"].GetInt());
            }
            if ( e1.HasMember("Tgd") ) {
                node.setDouble("Tgd", e1["Tgd"].GetDouble());
            }
            if ( e1.HasMember("toc") ) {
                node.setLong("toc", e1["toc"].GetInt());
            }
            if ( e1.HasMember("af2") ) {
                node.setDouble("af2", e1["af2"].GetDouble());
            }
            if ( e1.HasMember("af1") ) {
                node.setDouble("af1", e1["af1"].GetDouble());
            }
            if ( e1.HasMember("af0") ) {
                node.setDouble("af0", e1["af0"].GetDouble());
            }
        } else if ( frame == 2 and d.HasMember("EPHEM2") ) {
            node.setBool("frame2", true);
            const rapidjson::Value& e2 = d["EPHEM2"];
            if ( e2.HasMember("IODE") ) {
                node.setLong("IODE", e2["IODE"].GetInt());
            }
            if ( e2.HasMember("Crs") ) {
                node.setDouble("Crs", e2["Crs"].GetDouble());
            }
            if ( e2.HasMember("deltan") ) {
                node.setDouble("deltan", e2["deltan"].GetDouble());
            }
            if ( e2.HasMember("M0") ) {
                node.setDouble("M0", e2["M0"].GetDouble());
            }
            if ( e2.HasMember("Cuc") ) {
                node.setDouble("Cuc", e2["Cuc"].GetDouble());
            }
            if ( e2.HasMember("e") ) {
                node.setDouble("e", e2["e"].GetDouble());
            }
            if ( e2.HasMember("Cus") ) {
                node.setDouble("Cus", e2["Cus"].GetDouble());
            }
            if ( e2.HasMember("sqrtA") ) {
                node.setDouble("sqrtA", e2["sqrtA"].GetDouble());
            }
            if ( e2.HasMember("toe") ) {
                node.setLong("toe", e2["toe"].GetInt());
            }
            if ( e2.HasMember("FIT") ) {
                node.setLong("FIT", e2["FIT"].GetInt());
            }
            if ( e2.HasMember("AODO") ) {
                node.setLong("AODO", e2["AODO"].GetInt());
            }
        } else if ( frame == 3 and d.HasMember("EPHEM3") ) {
            node.setBool("frame3", true);
            const rapidjson::Value& e3 = d["EPHEM3"];
            if ( e3.HasMember("IODE") ) {
                node.setLong("IODE", e3["IODE"].GetInt());
            }
            if ( e3.HasMember("IDOT") ) {
                node.setDouble("IDOT", e3["IDOT"].GetDouble());
            }
            if ( e3.HasMember("Cic") ) {
                node.setDouble("Cic", e3["Cic"].GetDouble());
            }
            if ( e3.HasMember("Omega0") ) {
                node.setDouble("Omega0", e3["Omega0"].GetDouble());
            }
            if ( e3.HasMember("Cis") ) {
                node.setDouble("Cis", e3["Cis"].GetDouble());
            }
            if ( e3.HasMember("i0") ) {
                node.setDouble("i0", e3["i0"].GetDouble());
            }
            if ( e3.HasMember("Crc") ) {
                node.setDouble("Crc", e3["Crc"].GetDouble());
            }
            if ( e3.HasMember("omega") ) {
                node.setDouble("omega", e3["omega"].GetDouble());
            }
            if ( e3.HasMember("Omegad") ) {
                node.setDouble("Omegad", e3["Omegad"].GetDouble());
            }
        }
    } else {
        printf("gpsd: unhandled class = %s\n", msg_class.c_str());
        printf("parse: %s\n", message.c_str());
    }
    return true;
}

bool gpsd_t::process_buffer() {
    if ( json_buffer.length() <= 2 ) {
        return false;
    }
    //printf("buffer len: %lu\n", json_buffer.length());
    int start = 0;
    int end = 0;
    //printf("gpsd buffer: %s\n", json_buffer.c_str());
    int level = 0;
    for ( unsigned int i = start; i < json_buffer.length(); i++ ) {
        if ( json_buffer[i] == '{' ) {
            //printf("  found '{' @ %d\n", i); 
            if ( level == 0 ) {
                start = i;
            }
            level++;
        }
        if ( json_buffer[i] == '}' ) {
            //printf("  found '}' @ %d\n", i);
            level--;
            if ( level == 0 ) {
                end = i;
                parse_message(json_buffer.substr(start, end-start+1));
                json_buffer.erase(0, end+1);
                break;
            }
        }
    }
    // if ( level > 0 ) {
    //     printf("gpsd: incomplete json: %s\n", json_buffer.c_str());
    // }
    
    // try to ensure some sort of sanity so the buffer can't
    // completely run away if something bad is happening.
    if ( json_buffer.length() > 16384 ) {
        json_buffer = "";
        return true;
    }
    return false;
}

float gpsd_t::read() {
    bool gps_data_valid = false;

    if ( !socket_connected ) {
	connect();
    }

    const int buf_size = 256;
    char buf[buf_size];
    int result;
    while ( (result = gpsd_sock.recv( buf, buf_size-1 )) > 0 ) {
	buf[result] = 0;
        json_buffer += buf;
    }
    if ( errno != EAGAIN ) {
	if ( display_on ) {
	    perror("gpsd_sock.recv()");
	}
	socket_connected = false;
    }

    process_buffer();
    
    // If more than 5 seconds has elapsed without seeing new data and
    // our last init attempt was more than 5 seconds ago, try
    // resending the init sequence.
    double gps_timestamp = gps_node.getDouble("timestamp");
    if ( get_Time() > gps_timestamp + 5 && get_Time() > last_init_time + 5 ) {
	send_init();
    }

    return gps_data_valid;
}

void gpsd_t::close() {
    gpsd_sock.close();
    socket_connected = false;
}
