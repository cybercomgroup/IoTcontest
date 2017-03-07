// var env = require('./env.js');
var dotenv = require('dotenv');
var PubNub = require('pubnub');


dotenv.load();

window.updateChart = function() {
  const pubnub = new PubNub({
    // publishKey : process.env.PUBNUB_PUBLISH_KEY,
    // subscribeKey : 'sub-c-66a216ea-f298-11e6-af0f-0619f8945a4f'
    subscribeKey : 'sub-c-e58317c4-fcd5-11e6-8240-0619f8945a4f'
  });

  eon.chart({
    pubnub: pubnub,
    channels: ["counterIoTUplink"],
    generate: {
      bindto: '#chart',
      data: {
        labels: true
      }
    },
    transform: function(data) {
      console.log(data);
      var obj = JSON.parse(data);

      console.log(obj.data);

      obj.data = obj.data + '==';
      console.log(obj.data);

      var buf = new Buffer(obj.data, 'base64');
      console.log(parseInt(buf.toString('hex'), 16));


      return {
        "eon": {
          "counter": parseInt(buf.toString('hex'), 16),
          "_eonDatetime": new Date().getTime()
        }
      }
    }
  });
}

window.updateAccumulatedChart = function() {
  const pubnub = new PubNub({
    // publishKey : process.env.PUBNUB_PUBLISH_KEY,
    // subscribeKey : 'sub-c-66a216ea-f298-11e6-af0f-0619f8945a4f'
    subscribeKey : 'sub-c-e58317c4-fcd5-11e6-8240-0619f8945a4f'
  });

  eon.chart({
    pubnub: pubnub,
    channels: ["accumulatedValueChannel"],
    generate: {
      bindto: '#accumulatedChart',
      data: {
        labels: true
      }
    },
    transform: function(data) {
      return {
        "eon": {
          "totalCounter": data.accumulatedValue,
          "_eonDatetime": new Date().getTime()
        }
      }
    }
  });
}
