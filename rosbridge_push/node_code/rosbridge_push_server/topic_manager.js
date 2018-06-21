/**
 * Manages topics.
 */
module.exports = () => {
  return ({
    managedTopics: new Map(),
    
    createManagedTopic(topicName, latched) {
      const managedTopic = {
        clientIds: [],
        latched: latched,
        lastMsg: false,

        setLastMsg(lastMsg) {
          this.lastMsg = lastMsg;
        },

        addClient(clientId) {
          this.clientIds.push(clientId);
        }
      };      
      this.managedTopics.set(topicName, managedTopic);
      return managedTopic;
    },

  });
}
