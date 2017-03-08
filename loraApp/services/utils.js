// var env = require('./env.js');
var dotenv = require('dotenv');
var PubNub = require('pubnub');


dotenv.load();

window.updateCounterChart = function() {
  const pubnub = new PubNub({
    subscribeKey : 'sub-c-e58317c4-fcd5-11e6-8240-0619f8945a4f'
  });

  eon.chart({
    pubnub: pubnub,
    channels: ["counterIoTUplink"],
    generate: {
      bindto: '#counterChart',
      data: {
        labels: true,
        type: "area-spline"
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

window.updateSpinsChart = function() {
  const pubnub = new PubNub({
    subscribeKey : 'sub-c-e58317c4-fcd5-11e6-8240-0619f8945a4f'
  });

  eon.chart({
    pubnub: pubnub,
    channels: ["counterIoTUplink"],
    generate: {
      bindto: '#spinsChart',
      data: {
        labels: true,
        type: "area-spline"
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

window.updateSpinsAccumulatedChart = function() {
  const pubnub = new PubNub({
    subscribeKey : 'sub-c-e58317c4-fcd5-11e6-8240-0619f8945a4f'
  });

  eon.chart({
    pubnub: pubnub,
    channels: ["accumulatedValueChannel"],
    generate: {
      bindto: '#spinsAccumulatedChart',
      data: {
        labels: true,
        type: "area-spline"
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

window.updateTable = function() {
  const pubnub = new PubNub({
    subscribeKey : 'sub-c-e58317c4-fcd5-11e6-8240-0619f8945a4f'
  });

  pubnub.addListener({
    status: function(statusEvent) {
      if (statusEvent.category === "handleData") {
        console.log("handle data");
      }
    },
    message: function(message) {
      var obj = JSON.parse(message.message);
      var buf = new Buffer(obj.data, 'base64');
      var currentValue = parseInt(buf.toString('HEX'), 16);

      var tableRef = document.getElementById('fallAccidentsTable');
      // Insert a row in the table at the last row
      var newRow   = tableRef.insertRow(tableRef.rows.length);

      // Insert a cell in the row at index 0
      var newCellCol0  = newRow.insertCell(0);
      var newCellCol1  = newRow.insertCell(1);

      // Append a text node to the cell
      var text  = document.createTextNode('New row column 0');
      newCellCol0.appendChild(text);

      // Append a text node to the cell
      text  = document.createTextNode('New row column 1');
      newCellCol1.appendChild(text);
    },
    presence: function(presenceEvent) {
      console.log("presence event");
    }
  })

  pubnub.subscribe({
      channels: ['counterIoTUplink']
  });
}
