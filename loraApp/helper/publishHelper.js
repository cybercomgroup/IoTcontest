var PubNub = require('pubnub');
const request = require('request-promise');
var Client = require('node-rest-client').Client;

const deviceType = 'explorer-devicetype';
const deviceId = 'loraSecond';
const token = '6k@bx3K2kWe_LtdY!x';
const event = 'myEvent';
const url = 'https://oefljl.internetofthings.ibmcloud.com/api/v0002/device/types/' + deviceType + '/devices/' + deviceId + '/events/' + event;
const username = "use-token-auth";
const auth = "Basic " + new Buffer(username + ":" + token).toString("base64");


function publish() {

    console.log("Subscribing..");
    var accumulatedValue = 0;

    var pubnub = new PubNub({
        publishKey : process.env.PUBNUB_PUBLISH_KEY,
        subscribeKey : process.env.PUBNUB_SUBSCRIBE_KEY
    })

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

            var objectToSend = {
                counter: currentValue
            };

            accumulatedValue += currentValue;

            pubnub.publish(
              {
                  message: {
                      accumulatedValue: accumulatedValue
                  },
                  channel: 'accumulatedValueChannel'
              },
              function (status, response) {
                  if (status.error) {
                      console.log(status)
                  } else {
                      console.log("message Published w/ timetoken", response.timetoken)
                  }
              }
            );

            // // var options_auth = { user: username, password: token };
            // var client = new Client();
            //
            // // set content-type header and data as json in args parameter
            // var args = {
            //     data: objectToSend,
            //     headers: {
            //       "Content-Type": "application/json",
            //       "Authorization": auth
            //     }
            // };
            //
            // client.post(url, args, function (data, response) {
            //     // parsed response body as js object
            //     console.log(response.statusCode);
            // });
        },
        presence: function(presenceEvent) {
            console.log("presence event");
        }
    })

    pubnub.subscribe({
        channels: ['counterIoTUplink']
    });
};

module.exports = publish;
