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
      this.uid++;

      ws.on('message', (msg) => {
        try {
          const msgObj = JSON.parse(msg);
          
          if (this.source) {
            let mngdTopic = this.topicMgr.managedTopics.get(msgObj.topic);

            if (mngdTopic === undefined) {
              console.log("Creating a new managed topic");
              this.source.send(msg);
              mngdTopic = this.topicMgr.createManagedTopic(msgObj.topic); 
            }           

            mngdTopic.addClient(ws.switchId);

	    // Send the last message for latched topics.
            if (mngdTopic.latched && mngdTopic.lastMsg) {
              console.log("Sending latched msg");
              this.send(mngdTopic.lastMsg, ws.switchId);
            }
          }
        } catch (err) { 
          if (err instanceof SyntaxError) {

          }
          console.log(err);
        }
      }); 

      ws.on('close', () => {
        this.topicMgr.removeClient(ws.switchId);
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
