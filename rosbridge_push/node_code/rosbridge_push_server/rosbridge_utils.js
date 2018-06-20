/**
 * Utility functions.
 */

module.exports.validMsg = (msgObj, op, topic) => {
  return (msgObj.hasOwnProperty('op') &&
          msgObj.hasOwnProperty('topic') &&
          msgObj.op == op &&
          msgObj.topic == topic);
};
