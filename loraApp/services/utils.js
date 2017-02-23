// var env = require('./env.js');
var dotenv = require('dotenv');
var PubNub = require('pubnub');

dotenv.load();

window.updateChart = function() {

  var pubnub = new PubNub({
    publishKey : process.env.PUBNUB_PUBLISH_KEY,
    subscribeKey : 'sub-c-66a216ea-f298-11e6-af0f-0619f8945a4f'
  });

  eon.chart({
    pubnub: pubnub,
    channels: ["Tele2Uplink"],
    generate: {
      bindto: '#chart',
      data: {
        labels: true
      }
    },
    transform: function(data) {
      var obj = JSON.parse(data);

      return {
        "eon": {
          "temp": atob(obj.data.toString()),
          "_eonDatetime": new Date().getTime()
        }
      }
    }
  });
}
