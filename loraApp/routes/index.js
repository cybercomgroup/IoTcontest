var express = require('express');
var router = express.Router();

var PubNub = require('pubnub');
const request = require('request-promise');
var Client = require('node-rest-client').Client;

const deviceType = 'explorer-devicetype';
const deviceId = 'loraSecond';
const token = 'me8MYwqHuiC-UF3j?N';
const event = 'myEvent';
const url = 'https://oefljl.internetofthings.ibmcloud.com/api/v0002/device/types/' + deviceType + '/devices/' + deviceId + '/events/' + event;
const username = "use-token-auth";
const auth = "Basic " + new Buffer(username + ":" + token).toString("base64");


function publish() {

    var pubnub = new PubNub({
        publishKey : 'pub-c-6a280745-fb91-471d-a9bd-117e5d47c756',
        subscribeKey : 'sub-c-66a216ea-f298-11e6-af0f-0619f8945a4f'
    })

    pubnub.addListener({
        status: function(statusEvent) {
            if (statusEvent.category === "handleData") {
                console.log("handle data");
            }
        },
        message: function(message) {
            // var obj = JSON.parse(message.message);
            // var buf = new Buffer(obj.data, 'base64');
            //
            // var objectToSend = {
            //     temp: buf.toString()
            // };
            //
            // var options_auth = { user: username, password: token };
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

            // client.post(url, args, function (data, response) {
            //     // parsed response body as js object
            //     console.log(response.statusCode);
            // });
        },
        presence: function(presenceEvent) {
            console.log("presence event");
        }
    })
    console.log("Subscribing..");
    pubnub.subscribe({
        channels: ['Tele2Uplink']
    });
};

/* GET home page. */
router.get('/', function(req, res, next) {
    publish();

    res.render('index', { title: 'ejs' });
});

module.exports = router;
