#include "math.h"
#include "rapidjson/document.h"
#include "time.h"
#include "util/sg_path.h"

#include "gps_gpsd.h"
#include "raw_sat.h"

#include <iostream>
using std::cout;
using std::endl;

// connect to gpsd
void gpsd_t::connect() {
    // make sure it's closed
    gpsd_sock.close();

    if ( verbose ) {
	printf("Attempting to connect to gpsd @ %s:%d ... ",
	       host.c_str(), port);
    }

    if ( ! gpsd_sock.open( true ) ) {
	if ( verbose ) {
	    printf("error opening gpsd socket\n");
	}
	return;
    }
    
    if (gpsd_sock.connect( host.c_str(), port ) < 0) {
	if ( verbose ) {
	    printf("error connecting to gpsd\n");
	}
	return;
    }

    gpsd_sock.setBlocking( false );

    socket_connected = true;

    send_init();

    if ( verbose ) {
	printf("success!\n");
    }
}

// send our configured init strings to configure gpsd the way we prefer
void gpsd_t::send_init() {
    if ( !socket_connected ) {
	return;
    }

    if ( init_string != "" ) {
	if ( verbose ) {
	    printf("sending to gpsd: %s\n", init_string.c_str());
	}
	if ( gpsd_sock.send( init_string.c_str(), init_string.length() ) < 0 ) {
	    socket_connected = false;
	}
    }

    last_init_time = get_Time();
}

void gpsd_t::init( PropertyNode *config ) {
    if ( config->hasChild("port") ) {
	port = config->getInt("port");
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
    gps_node = PropertyNode(output_path.c_str(), true);
    string raw_path = get_next_path("/sensors", "gps_raw", true);
    raw_node = PropertyNode(raw_path.c_str(), true);
    ephem_node = raw_node.getChild("ephemeris", true);
}

bool gpsd_t::parse_message(const string message) {
    //printf("parse: %s\n", message.c_str());
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
            char* ptr = strptime(time_str.c_str(), "%Y-%m-%dT%H:%M:%S", &t);
            // printf("hour: %d min: %d sec: %d\n", t.tm_hour, t.tm_min, t.tm_sec);
            if ( ptr == nullptr ) {
                printf("gpsd: unable to parse time string = %s\n",
                       time_str.c_str());
            } else {
                double t2 = timegm(&t); // UTC
                if ( *ptr ) {
                    double fraction = atof(ptr);
                    // printf("fraction: %f\n", fraction);
                    t2 += fraction;
                }
                gps_node.setDouble( "unix_time_sec", t2 );
                gps_node.setDouble( "timestamp", get_Time() );
                // printf("%s %f\n", time_str.c_str(), t2);
            }
        }
        if ( d.HasMember("leapseconds") ) {
            leapseconds = d["leapseconds"].GetDouble();
            gps_node.setDouble( "leapseconds", leapseconds );
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
            gps_node.setInt( "fixType", d["mode"].GetInt() );
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
            gps_node.setInt( "satellites", num_sats );
        }
    } else if ( msg_class == "RAW" ) {
        double receiver_timestamp = 0.0;
        if ( d.HasMember("time") ) {
            // FIXME: these values need to be kept separate because a double
            // will lose precision in nanoseconds which affects accuracy!
            receiver_timestamp = d["time"].GetDouble();
            if ( d.HasMember("nsec") ) {
                receiver_timestamp += d["nsec"].GetDouble() / 1000000000.0;
            }
        }
        if ( d.HasMember("rawdata") ) {
            const rapidjson::Value& raw = d["rawdata"];
            if ( raw.IsArray() ) {
                raw_node.setInt("raw_num", raw.Size());
                raw_node.setDouble( "timestamp", get_Time() );
                raw_node.setDouble( "receiver_timestamp", receiver_timestamp);
                /* FIXME: */ double gps_seconds = receiver_timestamp - (315964800.0 - leapseconds);
                // /* FIXME: */ double gps_seconds = receiver_timestamp - (315964800.0 + leapseconds);
                // /* FIXME */ double gps_seconds = receiver_timestamp - 315964800.0;
                double tow = fmod(gps_seconds, 604800);
                printf("receiver timestamp: %.3f tow: %.3f\n", receiver_timestamp, tow);
                raw_node.setDouble("receiver_tow", tow);
                MatrixXd gnss(12,8);
                gnss.setZero();
                int mcount = 0;
                for (rapidjson::SizeType i = 0; i < raw.Size(); i++) {
                    int gnssid = -1;
                    int svid = -1;
                    bool l1c = false;
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
                    if ( raw[i].HasMember("obs") ) {
                        // printf("%s\n", raw[i]["obs"].GetString());
                        if ( (string)(raw[i]["obs"].GetString()) == "L1C" ) {
                            l1c = true;
                        }
                    }
                    if ( raw[i].HasMember("pseudorange") ) {
                        pr = raw[i]["pseudorange"].GetDouble();
                        {
                            // hack/test fixme/delete me
                            // test if modeling clock error helps position?
                            const double c = 299792458; // Speed of light in m/s
                            pr -= c*0.01;
                        }

                    }
                    if ( raw[i].HasMember("doppler") ) {
                        doppler = raw[i]["doppler"].GetDouble();
                    }
                    if ( gnssid == 0 ) {
                        PropertyNode ephem = ephem_node.getChild(id_str.c_str(), true);
                        if ( ! ephem.hasChild("frame1") ) {
                            ephem.setBool("frame1", false);
                        }
                        if ( ! ephem.hasChild("frame2") ) {
                            ephem.setBool("frame2", false);
                        }
                        if ( ! ephem.hasChild("frame3") ) {
                            ephem.setBool("frame3", false);
                        }
                        if ( l1c ) {
                            if (clockBiasEst_m != clockBiasEst_m) {
                                // catch nans
                                clockBiasEst_m = 0.0;
                            }
                            const double c = 299792458; // Speed of light in m/s

                            // correct pseudorange for clockbias (est) m
                            pr -= clockBiasEst_m;
                            double sat_trans_tow = tow + clockBiasEst_m/c - pr/c;
                            // /*combine terms*/ double sat_trans_tow = tow - (2*clockBiasEst_m - pr)/c;
                            
                            VectorXd posvel = dump_sat_pos(svid, sat_trans_tow, pr, doppler, ephem);
                            if ( posvel.size() == 8 ) {
                                // cout << "gnss:" << gnss << endl;
                                Vector3d me(-248211.09, -4500083.91, 4498382.30);
                                Vector3d diff = me - posvel.segment<3>(2);
                                double dist = sqrt(diff[0]*diff[0] + diff[1]*diff[1] + diff[2]*diff[2]);
                                printf("svid: %d pr %.2f geo %.2f diff: %.0f cbe: %.0f\n", svid, pr, dist, pr-dist, clockBiasEst_m);
                                gnss.row(mcount) = posvel;
                                mcount++;
                            }
                        }
                    }
                    string sat_name = "raw_satellite/" + std::to_string(i);
                    PropertyNode node = raw_node.getChild(sat_name.c_str(), true);
                    node.setInt("gnssid", gnssid);
                    node.setInt("svid", svid);
                    node.setBool("L1C", l1c);
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
                }
                if ( mcount >= 4 ) {
                    MatrixXd final_gnss(mcount,8);
                    final_gnss = gnss.block(0,0,mcount,8);
                    cout << "final gnss:" << final_gnss << endl;
                    pEst_E_m = Vector3d(0, 0, 0);
                    vEst_E_mps = Vector3d(0, 0, 0);
                    double clockBias_m = 0;
                    double clockRateBias_mps = 0;
                    GNSS_LS_pos_vel(final_gnss, pEst_E_m, vEst_E_mps, clockBias_m, clockRateBias_mps);
                    clockBiasEst_m += clockBias_m;
                    //clockBiasEst_m = clockBias_m;
                    printf("pos ecef: %.2f %.2f %.2f  cb: %.1f cbe: %.0f\n",
                           pEst_E_m[0], pEst_E_m[1], pEst_E_m[2], clockBias_m,
                           clockBiasEst_m);
                    Vector3d lla = E2D(pEst_E_m);
                    printf("receiver pos lla: %.8f %.8f %.1f\n", lla[0]*180.0/M_PI, lla[1]*180.0/M_PI, lla[2]);
                    // Vector3d me(-248211.09, -4500083.91, 4498382.30);
                    // GNSS_clock_bias(final_gnss, me);
                } else {
                    printf("waiting for enough ephemeris and satellite data...\n");
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
        PropertyNode node = ephem_node.getChild(id_str.c_str(), true);
        int frame = 0;
        if ( d.HasMember("TOW17") ) {
            node.setInt( "TOW17", d["TOW17"].GetInt() );
        }
        if ( d.HasMember("frame") ) {
            frame = d["frame"].GetInt();
        }
        if ( frame == 1 and d.HasMember("EPHEM1") ) {
            node.setBool("frame1", true);
            const rapidjson::Value& e1 = d["EPHEM1"];
            if ( e1.HasMember("WN") ) {
                node.setInt("WN", e1["WN"].GetInt());
            }
            if ( e1.HasMember("IODC") ) {
                node.setInt("IODC", e1["IODC"].GetInt());
            }
            if ( e1.HasMember("L2") ) {
                node.setInt("L2", e1["L2"].GetInt());
            }
            if ( e1.HasMember("ura") ) {
                node.setInt("ura", e1["ura"].GetInt());
            }
            if ( e1.HasMember("hlth") ) {
                node.setInt("hlth", e1["hlth"].GetInt());
            }
            if ( e1.HasMember("L2P") ) {
                node.setInt("L2P", e1["L2P"].GetInt());
            }
            if ( e1.HasMember("Tgd") ) {
                node.setDouble("Tgd", e1["Tgd"].GetDouble());
            }
            if ( e1.HasMember("toc") ) {
                node.setInt("toc", e1["toc"].GetInt());
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
                node.setInt("IODE", e2["IODE"].GetInt());
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
                node.setInt("toe", e2["toe"].GetInt());
            }
            if ( e2.HasMember("FIT") ) {
                node.setInt("FIT", e2["FIT"].GetInt());
            }
            if ( e2.HasMember("AODO") ) {
                node.setInt("AODO", e2["AODO"].GetInt());
            }
        } else if ( frame == 3 and d.HasMember("EPHEM3") ) {
            node.setBool("frame3", true);
            const rapidjson::Value& e3 = d["EPHEM3"];
            if ( e3.HasMember("IODE") ) {
                node.setInt("IODE", e3["IODE"].GetInt());
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
        // save ephemeris as a json file (at most every 60 seconds)
        double t = get_Time();
        if ( t > ephem_write_time + 60 ) {
            PropertyNode logging_node = PropertyNode( "/config/logging", true );
            SGPath jsonfile = logging_node.getString("flight_dir");
            jsonfile.append( "ephemeris.json" );
            ephem_node.save(jsonfile.c_str());
            ephem_write_time = t;
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
	if ( verbose ) {
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

VectorXd gpsd_t::dump_sat_pos(int svid, double tow, double pr, double doppler,
                              PropertyNode ephem) {
    VectorXd posvel;
    // if ( ephem.getBool("frame1") and ephem.getBool("frame2")
    //      and ephem.getBool("frame3") ) {
    //     posvel = EphemerisData2Satecef( tow, 0 /*TOW*/, 0 /*L2*/, 0 /*week no*/, 0 /*l2_flag*/, 0 /*sv_acc*/, 0 /*sv_health*/,
    //                                     0 /*t_gd*/, 0 /*iodc*/, 0 /*t_oc*/, 0 /*a_f2*/, 0 /*a_f1*/, 0 /*a_f0*/,
    //                                     0 /*iode*/, ephem.getDouble("Crs"), ephem.getDouble("deltan")*M_PI, ephem.getDouble("M0")*M_PI, ephem.getDouble("Cuc"), ephem.getDouble("e"), ephem.getDouble("Cus"),
    //                                     ephem.getDouble("sqrtA"), ephem.getDouble("toe"), ephem.getDouble("Cic"), ephem.getDouble("Omega0")*M_PI, ephem.getDouble("Cis"), ephem.getDouble("i0")*M_PI, ephem.getDouble("Crc"),
    //                                     ephem.getDouble("omega")*M_PI, ephem.getDouble("Omegad")*M_PI, ephem.getDouble("IDOT")*M_PI );
    //     cout << "sat: " << svid << " pos/vel: " << posvel << endl;
    // }
    
    if ( ephem.getBool("frame1") and ephem.getBool("frame2")
         and ephem.getBool("frame3") ) {
        GNSS_raw_measurement gnss;
        gnss.timestamp = tow;
        gnss.pseudorange = pr;
        gnss.doppler = doppler;
        gnss.Crs = ephem.getDouble("Crs");
        gnss.deltan = ephem.getDouble("deltan")*M_PI;
        gnss.M0 = ephem.getDouble("M0")*M_PI;
        gnss.Cuc = ephem.getDouble("Cuc");
        gnss.e = ephem.getDouble("e");
        gnss.Cus = ephem.getDouble("Cus");
        gnss.sqrtA = ephem.getDouble("sqrtA");
        gnss.toe = ephem.getDouble("toe");
        gnss.Cic = ephem.getDouble("Cic");
        gnss.Omega0 = ephem.getDouble("Omega0")*M_PI;
        gnss.Cis = ephem.getDouble("Cis");
        gnss.i0 = ephem.getDouble("i0")*M_PI;
        gnss.Crc = ephem.getDouble("Crc");
        gnss.omega = ephem.getDouble("omega")*M_PI;
        gnss.Omegad = ephem.getDouble("Omegad")*M_PI;
        gnss.IDOT = ephem.getDouble("IDOT")*M_PI;
        posvel = EphemerisData2PosVelClock(gnss);
        // cout << "sat: " << svid << " pos/vel: " << posvel << endl;
        Vector3d ecef = posvel.segment<3>(2);
        Vector3d lla = E2D(ecef);
        printf("sat %d lla: %.8f %.8f %.1f\n", svid, lla[0]*180.0/M_PI, lla[1]*180.0/M_PI, lla[2]);
    }
    return posvel;
}
