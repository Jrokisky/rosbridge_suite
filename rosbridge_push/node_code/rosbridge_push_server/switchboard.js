"use strict";

/**
 * Return a switchboard operator.
 */

const rosbridge_utils = require('./rosbridge_utils');
const topicManager = require("./topic_manager");

module.exports = () => {
  return ({
    uid: 0,
    clients: new Map(),
    source: null,
    topicMgr: topicManager(),

    initClient(ws) {
      ws.switchId = this.uid;
      this.clients.set(ws.switchId, ws);
      // Semaphore to avoid race conditions?
      this.uid++;

      ws.on('message', (msg) => {
        try {
          const topic = '/map';
          const latched = true;

          const msgObj = JSON.parse(msg);
          
          if (rosbridge_utils.validMsg(msgObj, 'subscribe', topic)) {
            let mngdTopic = this.topicMgr.managedTopics.get(topic);
            if (mngdTopic === undefined) {
              console.log("Creating a new managed topic");
              mngdTopic = this.topicMgr.createManagedTopic(topic, latched); 
              // What if source does not actually create the topic?
              // Shouldn't that be linked to the topic manager?
              this.source.send(msg);
            }           
            mngdTopic.addClient(ws.switchId);

            if (mngdTopic.latched && mngdTopic.lastMsg) {
              console.log("sending latched message");
              this.send(mngdTopic.lastMsg, ws.switchId);
            }
          } else {
            Error('Invalid message');
          }
        } catch (err) { 
          if (err instanceof SyntaxError) {

          }
          console.log(err);
        }
      }); 
    },
	

    initSource(ws) {
      this.source = ws;
      ws.on('message', (msg) => {
        const msgObj = JSON.parse(msg);
        if (msgObj.hasOwnProperty('topic')) {
          this.broadcast(msgObj.topic, msg);
        }
      });
    },

    send(msg, clientId) {
      const client = this.clients.get(clientId);
      if (client !== undefined) {
        client.send(msg);
      } else {
        console.log("Client: " + clientId + " does not exist");
      }
    },

    broadcast(topicName, msg) {
      const mngdTopic = this.topicMgr.managedTopics.get(topicName);
      if (mngdTopic !== undefined) {
        mngdTopic.setLastMsg(msg);
        mngdTopic.clientIds.map(function(clientId, idx) {
          this.send(msg, clientId);
        }, this);
      }
    },

  });
};
