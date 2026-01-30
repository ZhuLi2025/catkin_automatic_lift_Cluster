#include "mqtt_client.hpp"

const std::string SERVER_ADDRESS("tcp://192.168.18.23:1883");
//const std::string DFLT_SERVER_ADDRESS { "tcp://192.168.1.23:1883" };
const std::string CLIENT_ID("truck_crane-01");
const std::string PUBTOPIC1("mqtt/telemetry1");
const std::string PUBTOPIC2("mqtt/telemetry2");
const char* PAYLOAD1 = "Hello World!";
const char* PAYLOAD2 = "Hi there!";
const char* PAYLOAD3 = "Is anyone listening?";
const char* PAYLOAD4 = "Someone is always listening.";
const char* LWT_PAYLOAD = "Last will and testament.";

//
const int	N_RETRY_ATTEMPTS = 5;

// The QoS for sending data
const int QOS = 1;

// How often to sample the "data"
const auto SAMPLE_PERIOD = milliseconds(50);

// How much the "data" needs to change before we publish a new value.
const int DELTA_MS = 100;

// How many to buffer while off-line
const int MAX_BUFFERED_MESSAGES = 1200;

// --------------------------------------------------------------------------
// Gets the current time as the number of milliseconds since the epoch:
// like a time_t with ms resolution.

const auto TIMEOUT = seconds(10);


mqtt_client::mqtt_client()
{
    // A client that just publishes normally doesn't need a persistent
    // session or Client ID unless it's using persistence, then the local
    // library requires an ID to identify the persistence files.

    cerr << "Initializing for server '" << SERVER_ADDRESS << "'..." << endl;
    client = new mqtt::async_client(SERVER_ADDRESS, CLIENT_ID);
    mqtt::connect_options connOpts;
    connOpts.set_clean_session(true);
    cb = new callback(*client, connOpts);
    client->set_callback(*cb);
    //connect(cb, &callback::sendMsgToClient, this, &mqtt_client::resolvePubData);
    connect(cb, &callback::reSub, this, &mqtt_client::mqtt_sub);

    cerr << "Initializing  ...OK" << endl;

    // Start the connection.
    // Waiting for the connection untill completed.
    try {
      cerr << "\nConnecting..." << endl;
      mqtt::token_ptr conntok = client->connect(connOpts);
      cerr << "Waiting for the connection..." <<conntok<< endl;
      conntok->wait();
      cerr << "connection  ...OK" << endl;
      //mqtt_sub();
    }
    catch (const mqtt::exception& exc) {
      std::cerr << "\nERROR: Unable to connect to MQTT server: '"
        << SERVER_ADDRESS << "'" << exc << std::endl;  // exc <<
    }
    //mqtt_sub();

    //connect(cb, &callback::sendMsgToClient, this, &mqtt_client::resolvePubData);

}

void mqtt_client::mqtt_pub(){
  //publish msg
    try {
      while (true) {
        auto top1 = mqtt::topic(*client, PUBTOPIC1, QOS);
        auto top2 = mqtt::topic(*client, PUBTOPIC2, QOS);
        top2.publish(PAYLOAD2);
        top1.publish(PAYLOAD1);
        this_thread::sleep_for(std::chrono::milliseconds(100));
        cerr << "Sending message..." << endl;
      }
    }
    catch (const mqtt::exception& exc) {
      cerr << "Pub Error" << exc.what() << endl;
      //return 1;
    }
}
void mqtt_client::mqtt_pub(const std::string& TOPIC, int QOS, const std::string& PAYLOAD){
  //publish msg
    try {
      auto top1 = mqtt::topic(*client, TOPIC, QOS);
      top1.publish(PAYLOAD);
    }
    catch (const mqtt::exception& exc) {
      cerr << "Pub Error" << exc.what() << endl;
      //return 1;
    }
}
//dingyue
void mqtt_client::mqtt_sub(){
  //client.subscribe(TOPIC1, QOS, nullptr, subListener_);
  //client.subscribe(TOPIC2, QOS, nullptr, subListener_);
  //cerr << "cb->add_subTopic(TOPIC1, QOS);...start" << endl;
//  cb->add_subTopic(TOPIC1, QOS);
//  cb->add_subTopic(TOPIC2, QOS);
  cb->add_subTopic("command/avoid/truck_crane-01", 1);
  cb->add_subTopic("command/truck_crane-01", 1);
  cb->add_subTopic("task/crane/truck_crane-01", 1);
  cb->add_subTopic("command/ack1/icrane-02",1);
  cb->add_subTopic("state/autolifting/truck_crane-01",1);
  cb->add_subTopic("broadcast/received",1);

  cb->add_subTopic("task/crane/crane-01",1);//语音
  
  //std::map<string, int>::iterator  iterd = subTopicMap.begin();
//  while(iterd != subTopicMap.end()){
//    cb->add_subTopic(iterd->first, iterd->second);
//    iterd++;
//  }
}

void mqtt_client::mqtt_disconnect(){
  // Disconnect
  try {
    std::cerr << "\nDisconnecting from the MQTT server..." << std::flush;
    client->disconnect()->wait();
    std::cerr << "OK" << std::endl;
  }
  catch (const mqtt::exception& exc) {
    std::cerr << "disconnect Error" << exc << std::endl;
    //return 1;
  }
}

/*void mqtt_client::resolvePubData(mqtt::const_message_ptr msg){
  qRegisterMetaType<mqtt::const_message_ptr>("mqtt::const_message_ptr");
  std::cerr << "Message arrived mqtt_client" << std::endl;
  //std::cerr << "\ttopic: '" << msg->get_topic() << "'" << std::endl;
  //std::cerr << "\tpayload: '" << msg->to_string() << "'\n" << std::endl;
  emit mqtt_senddata(msg);
}*/
