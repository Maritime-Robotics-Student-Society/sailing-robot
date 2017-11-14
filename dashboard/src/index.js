/////////////////////////////////////////
//          Web Socket Stuff           //
/////////////////////////////////////////
const ros = new ROSLIB.Ros({
  url: 'ws://192.168.12.1:8448/updates'
});

ros.on('connection', function() {
  console.log('Connected to websocket server.');
});

ros.on('error', function (error) {
  console.log('Error connecting to websocket server:', error);
});

ros.on('close', function () {
  console.log('Connection to websocket server closed.');
});

const ws = new WebSocket(`ws://192.168.12.1:8448/updates`);
ws.onopen = function (ws, event) {
  connectionOverlays.isDisconnected = false;
  connectionOverlays.isConnecting = false;
};
ws.onmessage = function (event) {
  const jsonMsg = JSON.parse(event.data);
  if (jsonMsg.topic === '/rosout') {
    rosout.addNew(jsonMsg);
  } else {
    topicsTable.update(jsonMsg);
    const updateCompassHand = topicHandlers[jsonMsg.topic];
    if (updateCompassHand) {
      updateCompassHand(jsonMsg.value);
    }
  }
};
ws.onerror = function (error) {
  connectionOverlays.isConnecting = false;
  connectionOverlays.isDisconnected = true;
}
ws.onclose = function (event) {
  connectionOverlays.isConnecting = false;
  connectionOverlays.isDisconnected = true;
};

/////////////////////////////////////////
//          Connection Alerts          //
/////////////////////////////////////////
const connectionOverlays = new Vue({
  el: '#connection-overlays',
  data: {
    isDisconnected: false,
    isConnecting: true
  }
});

/////////////////////////////////////////
//            Topics Table             //
/////////////////////////////////////////
const topicsTable = new Vue({
  el: '#topics-table',
  data: {
    headers: [
      'Topic',
      'Value'
    ],
    // topics: fp.range(0, 10).map((_, i) => {
    //   return Object.assign(Object.create(null), {
    //     name: `Test${Math.round(10 * Math.random())}`,
    //     value: 30 * Math.random()
    //   });
    // })
    topics: []
  },
  computed: {
    sortedTopics() {
      return this.topics.sort((a, b) => {
        return (a.name.localeCompare(b.name) > 0) ? 1 :
          (a.name.localeCompare(b.name === 0)) ? 0 :
          -1
      })
    }
  },
  methods: {
    /**
     * Formats the value in `msg`. Only used by `update()`.
     * @param {[key: string]: any} msg The message object to format
     */
    formatValue(msg) {
      if (msg.latitude !== undefined) {
        const latHemi = msg.latitude > 0 ? 'N' : 'S';
        const lonHemi = msg.longitude > 0 ? 'E' : 'W';
        return `${Math.abs(msg.latitude)}° ${latHemi} / ${Math.abs(msg.longitude)}°
       ${lonHemi}}`;
      } else {
        return (typeof msg.value === 'number')?
          msg.value.toFixed(2) : `${msg.value}`;
      }
    },
    /**
     * Updates the topics table with `msg`, the new message.
     * @param {[key: string]: any} msg The new message to update the table with
     */
    update(msg) {
      const topicName = msg.topic;
      const topic = this.topics.find(t => t.name === topicName);
      if (topic) {
        topic.value = this.formatValue(msg);
      } else {
        // Add new row to table
        this.topics.push({
          name: topicName,
          value: this.formatValue(msg)
        });
      }
    }
  }
});

/////////////////////////////////////////
//            Magic Compass            //
/////////////////////////////////////////
const CompassTable = Vue.component('compass-table', {
  props: {
    compasses: {
      type: Array,
      default: []
    }
  },
  template: `<table class="table table-striped table-bordered">
              <thead class="thead-inverse">
                <th>Hand</th>
                <th>Bearing</th>
              </thead>
              <tbody>
                <tr v-for="c of compasses">
                  <th scope="row">{{c.name.replace(/_.+/, '')}}</th>
                  <td>{{c.bearing}}°</td>
                </tr>
              </tbody>
            </table>`
});

const CompassHand = Vue.component('compass-hand', {
  props: {
    compass: {
      type: Object,
      default: Object.create(null)
    }
  },
  template: `<img :src="source"
                  :style="spinEm"
                  :alt="altName"
                  class="compass-hand">`,
  computed: {
    source() {
      return `static/${this.compass.name}.svg`;
    },
    altName() {
      return `Why isn't ${this.compass.name} loaded`;
    },
    spinEm() {
      return Object.assign(Object.create(null), {
        transform: `rotate(${this.compass.bearing}deg)`
      });
    }
  }
});

// This houses a CompassTable and CompassHands
const magicCompass = new Vue({
  el: '#magic-compass',
  data: {
    compasses: [{
        name: 'goal_hand',
        bearing: 0
      },
      {
        name: 'heading_hand',
        bearing: 0
      },
      {
        name: 'waypoint_hand',
        bearing: 0
      },
      {
        name: 'wind_hand',
        bearing: 0
      },
    ]
  },
  methods: {
    /**
     * Updates the value of the compass hand with name `name` to `bearing`.
     * @param {string} name The name of the compass hand
     * @param {number} bearing The new bearing to update `name` with
     */
    change(name, bearing) {
      const c = this.compasses.find(c => c.name.replace(/_.*$/, '') === name);
      if (!c) {
        console.log(`Can't find compass! Name: ${name}`);
      } else {
        c.bearing = (bearing % 360).toFixed(2);
      }
    }
  },
  components: {
    compassTable: CompassTable,
    compassHand: CompassHand
  }
})

/////////////////////////////////////////
//           /rosout Logging           //
/////////////////////////////////////////
const RosoutTable = Vue.component('rosout-table', {
  props: ['displayLevel', 'newMessage'],
  template: `<div>
    <p v-for="m of getMessages" :class="logLevel(m)">
      {{m.msg}}
    </p>
  </div>`,
  data() {
    return {
      recentMessages: [],
      recentMessagesMax: 1000, // Maximum number of messages to store in memory
    }
  },
  computed: {
    getMessages() {
      // Check if message contained in 'new-message' prop is a new message
      if (this.newMessage !== this.recentMessages[this.recentMessages.length - 1]) {
        this.appendMsg(this.newMessage);
      }
      return this.recentMessages
        .filter(m => m.level >= this.displayLevel);
    }
  },
  methods: {
    logLevel(m) {
      // Apply the appropriate colour to the log message
      return {
        debug: 1 & m.level,
        info: 2 & m.level,
        warn: 4 & m.level,
        error: 8 & m.level,
        fatal: 16 & m.level,
      }
    },
    appendMsg(m) {
      // Trim off the excess fats, plus an extra layer as we're about to add a
      // fresh new piece
      if (this.recentMessages.length > this.recentMessagesMax) {
        const discard = this.recentMessages.length - this.recentMessagesMax;
        this.recentMessages = this.recentMessages.slice(discard + 1);
      }
      // Apply the new layer on top
      this.recentMessages.push(m);
      // Allow the flavours to rise to the top
      if (this.$el) {
        this.$el.scrollTop = this.$el.scrollHeight;
      }
    }
  }
});

const rosout = new Vue({
  el: '#rosout',
  data: {
    levels: [
      'Debug', // 1
      'Info', // 2
      'Warn', // 4
      'Error', // 8
      'Fatal' // 16
    ],
    activeLevel: 1,
    newMessage: Object.create(null),
    pauseLogging: false, // If true, stop auto-scrolling of log table
    pauseLoggingText: 'Pause logging'
  },
  methods: {
    levelClass(currentLevel) {
      return (this.activeLevel === currentLevel) ? 'active' : '';
    },
    addNew(message) {
      // Feed new message to 'new-message' prop if pauseLogging is false
      if (!this.pauseLogging) this.newMessage = message;
    },
    togglePause() {
      this.pauseLogging = !this.pauseLogging;
      this.pauseLoggingText = (this.pauseLogging === true) ? 'Resume logging' :
          'Pause logging';
    }
  },
  components: {
    rosoutTable: RosoutTable
  }
});

/////////////////////////////////////////
//            Topic Handlers           //
/////////////////////////////////////////
const topicHandlers = {
  '/heading': fp.curry(magicCompass.change, 'heading'),
  '/goal_heading': fp.curry(magicCompass.change, 'goal'),
  '/dbg_heading_to_waypoint': fp.curry(magicCompass.change, 'waypoint'),
  '/wind_direction_average': fp.curry(magicCompass.change, 'wind')
};
