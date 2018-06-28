/**
 * Manages topics.
 */
module.exports = () => {
  return ({
    managedTopics: new Map(),
    latchedTopics: ['/map'],
    
    createManagedTopic(topicName) {
      const latched = this.latchedTopics.indexOf(topicName) > -1;

      const managedTopic = {
        clientIds: [],
        latched: latched,
        lastMsg: false,

        setLastMsg(lastMsg) {
          this.lastMsg = lastMsg;
        },

        addClient(clientId) {
          this.clientIds.push(clientId);
        },

        removeClient(clientId) {
          let idx = this.clientIds.indexOf(clientId);
          if (idx > -1) {
            this.clientIds.splice(idx, 1);
          }
        },
      };      

      this.managedTopics.set(topicName, managedTopic);
      return managedTopic;
    },

    removeClient(clientId) {
      this.managedTopics.forEach((mngdTopic, key, map) => {
        mngdTopic.removeClient(clientId);
      });
      // TODO: need to handle unsubscribe
    },

  });
}
