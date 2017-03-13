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
    subscribeKey : 'sub-c-3be6e3c4-fcc8-11e6-99d2-0619f8945a4f'
  });

  eon.chart({
    pubnub: pubnub,
    channels: ["bikeIoTUplink"],
    generate: {
      bindto: '#spinsChart',
      data: {
        labels: true,
        type: "area-spline"
      }
    },
    transform: function(data) {

      var obj = JSON.parse(data);
      var buf = new Buffer(obj.data, 'base64');
      obj = JSON.parse(buf.toString());

      if (obj.spins) {
        var currentValue = obj.spins;

        return {
          "eon": {
            "counter": currentValue,
            "_eonDatetime": new Date().getTime()
          }
        }
      }
    }
  });
}

window.updateSpinsAccumulatedChart = function() {
  const pubnub = new PubNub({
    subscribeKey : 'sub-c-3be6e3c4-fcc8-11e6-99d2-0619f8945a4f'
  });

  eon.chart({
    pubnub: pubnub,
    channels: ["bikeAccumulatedValueChannel"],
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
    publishKey : 'pub-c-1d98f83e-a179-4d49-b022-41b2336bb0a6',
    subscribeKey : 'sub-c-3be6e3c4-fcc8-11e6-99d2-0619f8945a4f'
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
      obj = JSON.parse(buf.toString());

      if (obj.fall) {
        var tableRef = document.getElementById('fallAccidentsTable');
        // Insert a row in the table at the last row
        var newRow   = tableRef.insertRow(tableRef.rows.length);

        // Insert a cell in the row at index 0
        var newCellCol0  = newRow.insertCell(0);
        var newCellCol1  = newRow.insertCell(1);
        var newCellCol2  = newRow.insertCell(2);

        // Append a text node to the cell
        var text  = document.createTextNode(new Date());
        newCellCol0.appendChild(text);

        // Append a text node to the cell
        var text  = document.createTextNode(obj.la);
        newCellCol1.appendChild(text);

        // Append a text node to the cell
        text  = document.createTextNode(obj.lo);
        newCellCol2.appendChild(text);

        var createClickHandler = function(row) {
          return function() {
            var col1 = row.getElementsByTagName("td")[1];
            var col2 = row.getElementsByTagName("td")[2];
            var lat = col1.innerHTML;
            var lng = col2.innerHTML;

            pubnub.publish(
              {
                message: {
                    lat: lat,
                    lng: lng
                },
                channel: 'bikeLocationToMap'
              },
              function (status, response) {
                if (status.error) {
                    console.log(status)
                } else {
                    console.log("message Published w/ timetoken", response.timetoken)
                }
              }
            );
         };
        };

        newRow.onclick = createClickHandler(newRow);
      }
    },
    presence: function(presenceEvent) {
      console.log("presence event");
    }
  })

  pubnub.subscribe({
      channels: ['bikeIoTUplink']
  });
}
