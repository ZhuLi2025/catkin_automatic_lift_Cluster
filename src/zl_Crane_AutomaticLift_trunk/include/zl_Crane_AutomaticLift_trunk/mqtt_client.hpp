#ifndef MQTT_CLIENT_H
#define MQTT_CLIENT_H

#include <QObject>
#include <iostream>
#include <cstdlib>
#include <string>
#include <cstring>
#include <cctype>
#include <thread>
#include <chrono>
#include <atomic>
#include "mqtt/async_client.h"
#include <QMetaType>

using namespace std;
using namespace std::chrono;

/////////////////////////////////////////////////////////////////////////////
class action_listener : public virtual mqtt::iaction_listener
{
  std::string name_;
protected:
  void on_failure(const mqtt::token& tok) override {
    std::cerr << name_ << " failure";
    if (tok.get_message_id() != 0)
      std::cerr << " for token: [" << tok.get_message_id() << "]" << std::endl;
    std::cerr << std::endl;
  }

  void on_success(const mqtt::token& tok) override {
    std::cerr << name_ << " success";
    if (tok.get_message_id() != 0)
      std::cerr << " for token: [" << tok.get_message_id() << "]" << std::endl;
    auto top = tok.get_topics();
    if (top && !top->empty())
      std::cerr << "\ttoken topic: '" << (*top)[0] << "', ..." << std::endl;
    std::cerr << std::endl;
  }

public:
  action_listener(const std::string& name) : name_(name) {}
};
/////////////////////////////////////////////////////////////////////////////
/**
 * Local callback & listener class for use with the client connection.
 * This is primarily intended to receive messages, but it will also monitor
 * the connection to the broker. If the connection is lost, it will attempt
 * to restore the connection and re-subscribe to the topic.
 */
class callback : public QObject,public virtual mqtt::callback,
          public virtual mqtt::iaction_listener

{
  Q_OBJECT
  // Counter for the number of connection retries
  int nretry_;
  // The MQTT client
  mqtt::async_client& cli_;
  // Options to use if we need to reconnect
  mqtt::connect_options& connOpts_;
  // An action listener to display the result of actions.
  action_listener subListener_;
signals:
  void sendMsgToClient(mqtt::const_message_ptr msg);
  void reSub();
private:
  // This deomonstrates manually reconnecting to the broker by calling
  // connect() again. This is a possibility for an application that keeps
  // a copy of it's original connect_options, or if the app wants to
  // reconnect with different options.
  // Another way this can be done manually, if using the same options, is
  // to just call the async_client::reconnect() method.
  void reconnect() {
    std::this_thread::sleep_for(std::chrono::milliseconds(2500));
    try {
      cli_.connect(connOpts_, nullptr, *this);
    }
    catch (const mqtt::exception& exc) {
      std::cerr << "Error: " << exc.what() << std::endl;
      //exit(1);
    }
  }

  // Re-connection failure
  void on_failure(const mqtt::token& tok) override {
    std::cerr << "Connection attempt failed" << std::endl;
    //if (++nretry_ > N_RETRY_ATTEMPTS)
    //  exit(1);
    reconnect();
  }

  // (Re)connection success
  // Either this or connected() can be used for callbacks.
  void on_success(const mqtt::token& tok) override {}

  // (Re)connection success
  void connected(const std::string& cause) override {
    std::cerr << "\nConnection success" << std::endl;
    emit reSub();
  }

  // Callback for when the connection is lost.
  // This will initiate the attempt to manually reconnect.
  void connection_lost(const std::string& cause) override {
    std::cerr << "\nConnection lost" << std::endl;
    if (!cause.empty())
      std::cerr << "\tcause: " << cause << std::endl;

    std::cerr << "Reconnecting..." << std::endl;
    nretry_ = 0;
    reconnect();
  }
//public:
  // Callback for when a message arrives.
  void message_arrived(mqtt::const_message_ptr msg) override {
    std::cerr << "Message arrived" << std::endl;
    //std::cerr << "\ttopic: '" << msg->get_topic() << "'" << std::endl;
    //std::cerr << "\tpayload: '" << msg->to_string() << "'\n" << std::endl;
    emit sendMsgToClient(msg);
  }

  void delivery_complete(mqtt::delivery_token_ptr token) override {}

public:
  callback(mqtt::async_client& cli, mqtt::connect_options& connOpts)
        : nretry_(0), cli_(cli), connOpts_(connOpts), subListener_("Subscription") {}
  void add_subTopic(const std::string& TOPIC, int QOS){
    cli_.subscribe(TOPIC, QOS, nullptr, subListener_);
    cerr << "Subscribe TOPIC " << TOPIC <<endl;
  }
};
//////////////////////////////////////////////////////
///  The mqtt_client class////////////////////////////
//////////////////////////////////////////////////////
///
class mqtt_client: public QObject
{
  Q_OBJECT
public:
  mqtt_client();
  void mqtt_pub();
  void mqtt_pub(const std::string& TOPIC, int QOS, const std::string& PAYLOAD);
  void mqtt_disconnect();

private:
  mqtt::async_client* client;
  //mqtt::connect_options* connOpts;
public:
  callback* cb;
  map<string, int> subTopicMap;   ////订阅topic

public slots:
  //void resolvePubData(mqtt::const_message_ptr msg);
  void mqtt_sub();

signals:
  //void mqtt_senddata(mqtt::const_message_ptr msg);


};

#endif // MQTT_CLIENT_H
