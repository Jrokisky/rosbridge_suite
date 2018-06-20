"use strict";

/**
 * Return a switchboard operator.
 */

const rosbridge_utils = require('./rosbridge_utils');

module.exports = () => {
  return ({
    uid: 0,
    clients: [],
    source: null,
    topics: new Map(),

    initClient(ws) {
      ws.switchId = this.uid;
      this.clients.push(ws);
      // Semaphore to avoid race conditions?
      this.uid++;

      ws.on('message', (msg) => {
        try {
          let msgObj = JSON.parse(msg);
          const topic = '/map';
          if (rosbridge_utils.validMsg(msgObj, 'subscribe', topic)) {

            let subscribedClients = this.topics.get(topic);
            if (subscribedClients === undefined) {
              subscribedClients = [];
              // TODO: how do we ensure that the source is initialized?? or send msgs when connection finally happens?
              // Pass msg along to source. Do we need to overwrite the id?
              // TODO: removing png compression for now so we don't need to decompress/recompress.
              delete msgObj.compression;
              let msg = JSON.stringify(msgObj);
              console.log("sending to source");
              console.log(msg);
              this.source.send(msg);
            }           
            subscribedClients.push(ws.switchId);
            this.topics.set(topic, subscribedClients);
          } else {
            Error('Invalid message');
          }
        } catch (err) { 
          if (err instanceof SyntaxError) {
            // TODO: handle json parsing error
          }
          console.log(err);
        }
      }); 
    },
	

    initSource(ws) {
      this.source = ws;
      ws.on('message', (msg) => {
        // TODO: pass msg along to clients.
        this.clients.map((client) => {
          client.send(msg);
        });
      });
    },

  });
};
