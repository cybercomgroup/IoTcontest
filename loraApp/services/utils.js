// var env = require('./env.js');
var dotenv = require('dotenv');
var PubNub = require('pubnub');


dotenv.load();

window.updateChart = function() {
  const pubnub = new PubNub({
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

      console.log(obj);

      return {
        "eon": {
          "temp": atob(obj.data.toString()),
          "_eonDatetime": new Date().getTime()
        }
      }
    }
  });
}

window.updateMap = function() {

  var pn = new PubNub({
    subscribeKey : 'sub-c-66a216ea-f298-11e6-af0f-0619f8945a4f',
    publishKey : 'pub-c-6a280745-fb91-471d-a9bd-117e5d47c756',
    ssl : true
  });

  eon.map({
    pubnub: pn,
    channels: ['Tele2Uplink'],
    id: 'map',
    mbId: 'mapbox.streets',
    // mbToken: 'pk.ey31IjoiaWRtc3giLCJhIjoiZZ1zMGI2ZjBlNTMxZjk5YTEwNjM5WNJlOWI4MmJiZGIifQ.U1jMQo2QVeuUtt85oD7hkQ'
    // mbToken: 'pk.eyJ1IjoiaWFuamVubmluZ3MiLCJhIjoiZExwb0p5WSJ9.XLi48h-NOyJOCJuu1-h-Jg',
    mbToken: 'pk.eyJ1IjoiYWJkdWxsYWFsY2hhbGF0aSIsImEiOiJjaXpwZ2szM2YwMDEwMzJteWtxaXFmMTVnIn0.0NEEQaZ8RkuFwEj9eh0kfg',
    // mbId: 'ianjennings.l896mh2e',

  });
}

// window.updateMap = function() {
//
//   const pubnub = new PubNub({
//     publishKey : process.env.PUBNUB_PUBLISH_KEY,
//     subscribeKey : 'sub-c-66a216ea-f298-11e6-af0f-0619f8945a4f'
//   });
//
//   eon.map({
//     pubnub: pubnub,
//     id: 'map',
//     mbToken: 'pk.eyJ1IjoiYWJkdWxsYWFsY2hhbGF0aSIsImEiOiJjaXpwZ2szM2YwMDEwMzJteWtxaXFmMTVnIn0.0NEEQaZ8RkuFwEj9eh0kfg',
//     mbId: 'ianjennings.l896mh2e',
//     channels: ["Tele2Uplink"]
//   });
// }
