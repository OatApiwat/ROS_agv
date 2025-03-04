/*!
 * @license EaselJS
 * Visit http://createjs.com/ for documentation, updates and examples.
 *
 * Copyright (c) 2011-2013 gskinner.com, inc.
 *
 * Distributed under the terms of the MIT license.
 * http://www.opensource.org/licenses/mit-license.html
 *
 * This notice shall be included in all copies or substantial portions of the Software.
 */
(this.createjs = this.createjs || {}),
  (function () {
    "use strict";
    var a = function (a, b, c) {
        this.initialize(a, b, c);
      },
      b = a.prototype;
    (b.type = null),
      (b.target = null),
      (b.currentTarget = null),
      (b.eventPhase = 0),
      (b.bubbles = !1),
      (b.cancelable = !1),
      (b.timeStamp = 0),
      (b.defaultPrevented = !1),
      (b.propagationStopped = !1),
      (b.immediatePropagationStopped = !1),
      (b.removed = !1),
      (b.initialize = function (a, b, c) {
        (this.type = a),
          (this.bubbles = b),
          (this.cancelable = c),
          (this.timeStamp = new Date().getTime());
      }),
      (b.preventDefault = function () {
        this.defaultPrevented = !0;
      }),
      (b.stopPropagation = function () {
        this.propagationStopped = !0;
      }),
      (b.stopImmediatePropagation = function () {
        this.immediatePropagationStopped = this.propagationStopped = !0;
      }),
      (b.remove = function () {
        this.removed = !0;
      }),
      (b.clone = function () {
        return new a(this.type, this.bubbles, this.cancelable);
      }),
      (b.toString = function () {
        return "[Event (type=" + this.type + ")]";
      }),
      (createjs.Event = a);
  })(),
  (this.createjs = this.createjs || {}),
  (function () {
    "use strict";
    var a = function () {},
      b = a.prototype;
    (a.initialize = function (a) {
      (a.addEventListener = b.addEventListener),
        (a.on = b.on),
        (a.removeEventListener = a.off = b.removeEventListener),
        (a.removeAllEventListeners = b.removeAllEventListeners),
        (a.hasEventListener = b.hasEventListener),
        (a.dispatchEvent = b.dispatchEvent),
        (a._dispatchEvent = b._dispatchEvent),
        (a.willTrigger = b.willTrigger);
    }),
      (b._listeners = null),
      (b._captureListeners = null),
      (b.initialize = function () {}),
      (b.addEventListener = function (a, b, c) {
        var d;
        d = c
          ? (this._captureListeners = this._captureListeners || {})
          : (this._listeners = this._listeners || {});
        var e = d[a];
        return (
          e && this.removeEventListener(a, b, c),
          (e = d[a]),
          e ? e.push(b) : (d[a] = [b]),
          b
        );
      }),
      (b.on = function (a, b, c, d, e, f) {
        return (
          b.handleEvent && ((c = c || b), (b = b.handleEvent)),
          (c = c || this),
          this.addEventListener(
            a,
            function (a) {
              b.call(c, a, e), d && a.remove();
            },
            f
          )
        );
      }),
      (b.removeEventListener = function (a, b, c) {
        var d = c ? this._captureListeners : this._listeners;
        if (d) {
          var e = d[a];
          if (e)
            for (var f = 0, g = e.length; g > f; f++)
              if (e[f] == b) {
                1 == g ? delete d[a] : e.splice(f, 1);
                break;
              }
        }
      }),
      (b.off = b.removeEventListener),
      (b.removeAllEventListeners = function (a) {
        a
          ? (this._listeners && delete this._listeners[a],
            this._captureListeners && delete this._captureListeners[a])
          : (this._listeners = this._captureListeners = null);
      }),
      (b.dispatchEvent = function (a, b) {
        if ("string" == typeof a) {
          var c = this._listeners;
          if (!c || !c[a]) return !1;
          a = new createjs.Event(a);
        }
        if (((a.target = b || this), a.bubbles && this.parent)) {
          for (var d = this, e = [d]; d.parent; ) e.push((d = d.parent));
          var f,
            g = e.length;
          for (f = g - 1; f >= 0 && !a.propagationStopped; f--)
            e[f]._dispatchEvent(a, 1 + (0 == f));
          for (f = 1; g > f && !a.propagationStopped; f++)
            e[f]._dispatchEvent(a, 3);
        } else this._dispatchEvent(a, 2);
        return a.defaultPrevented;
      }),
      (b.hasEventListener = function (a) {
        var b = this._listeners,
          c = this._captureListeners;
        return !!((b && b[a]) || (c && c[a]));
      }),
      (b.willTrigger = function (a) {
        for (var b = this; b; ) {
          if (b.hasEventListener(a)) return !0;
          b = b.parent;
        }
        return !1;
      }),
      (b.toString = function () {
        return "[EventDispatcher]";
      }),
      (b._dispatchEvent = function (a, b) {
        var c,
          d = 1 == b ? this._captureListeners : this._listeners;
        if (a && d) {
          var e = d[a.type];
          if (!e || !(c = e.length)) return;
          (a.currentTarget = this),
            (a.eventPhase = b),
            (a.removed = !1),
            (e = e.slice());
          for (var f = 0; c > f && !a.immediatePropagationStopped; f++) {
            var g = e[f];
            g.handleEvent ? g.handleEvent(a) : g(a),
              a.removed && (this.off(a.type, g, 1 == b), (a.removed = !1));
          }
        }
      }),
      (createjs.EventDispatcher = a);
  })(),
  (this.createjs = this.createjs || {}),
  (function () {
    "use strict";
    createjs.indexOf = function (a, b) {
      for (var c = 0, d = a.length; d > c; c++) if (b === a[c]) return c;
      return -1;
    };
  })(),
  (this.createjs = this.createjs || {}),
  (function () {
    "use strict";
    var a = function () {
      throw "UID cannot be instantiated";
    };
    (a._nextID = 0),
      (a.get = function () {
        return a._nextID++;
      }),
      (createjs.UID = a);
  })(),
  (this.createjs = this.createjs || {}),
  (function () {
    "use strict";
    var a = function () {
      throw "Ticker cannot be instantiated.";
    };
    (a.RAF_SYNCHED = "synched"),
      (a.RAF = "raf"),
      (a.TIMEOUT = "timeout"),
      (a.useRAF = !1),
      (a.timingMode = null),
      (a.maxDelta = 0),
      (a.removeEventListener = null),
      (a.removeAllEventListeners = null),
      (a.dispatchEvent = null),
      (a.hasEventListener = null),
      (a._listeners = null),
      createjs.EventDispatcher.initialize(a),
      (a._addEventListener = a.addEventListener),
      (a.addEventListener = function () {
        return !a._inited && a.init(), a._addEventListener.apply(a, arguments);
      }),
      (a._paused = !1),
      (a._inited = !1),
      (a._startTime = 0),
      (a._pausedTime = 0),
      (a._ticks = 0),
      (a._pausedTicks = 0),
      (a._interval = 50),
      (a._lastTime = 0),
      (a._times = null),
      (a._tickTimes = null),
      (a._timerId = null),
      (a._raf = !0),
      (a.init = function () {
        a._inited ||
          ((a._inited = !0),
          (a._times = []),
          (a._tickTimes = []),
          (a._startTime = a._getTime()),
          a._times.push((a._lastTime = 0)),
          a.setInterval(a._interval));
      }),
      (a.reset = function () {
        if (a._raf) {
          var b =
            window.cancelAnimationFrame ||
            window.webkitCancelAnimationFrame ||
            window.mozCancelAnimationFrame ||
            window.oCancelAnimationFrame ||
            window.msCancelAnimationFrame;
          b && b(a._timerId);
        } else clearTimeout(a._timerId);
        a.removeAllEventListeners("tick");
      }),
      (a.setInterval = function (b) {
        (a._interval = b), a._inited && a._setupTick();
      }),
      (a.getInterval = function () {
        return a._interval;
      }),
      (a.setFPS = function (b) {
        a.setInterval(1e3 / b);
      }),
      (a.getFPS = function () {
        return 1e3 / a._interval;
      }),
      (a.getMeasuredTickTime = function (b) {
        var c = 0,
          d = a._tickTimes;
        if (d.length < 1) return -1;
        b = Math.min(d.length, b || 0 | a.getFPS());
        for (var e = 0; b > e; e++) c += d[e];
        return c / b;
      }),
      (a.getMeasuredFPS = function (b) {
        var c = a._times;
        return c.length < 2
          ? -1
          : ((b = Math.min(c.length - 1, b || 0 | a.getFPS())),
            1e3 / ((c[0] - c[b]) / b));
      }),
      (a.setPaused = function (b) {
        a._paused = b;
      }),
      (a.getPaused = function () {
        return a._paused;
      }),
      (a.getTime = function (b) {
        return a._getTime() - a._startTime - (b ? a._pausedTime : 0);
      }),
      (a.getEventTime = function (b) {
        return (a._lastTime || a._startTime) - (b ? a._pausedTime : 0);
      }),
      (a.getTicks = function (b) {
        return a._ticks - (b ? a._pausedTicks : 0);
      }),
      (a._handleSynch = function () {
        var b = a._getTime() - a._startTime;
        (a._timerId = null),
          a._setupTick(),
          b - a._lastTime >= 0.97 * (a._interval - 1) && a._tick();
      }),
      (a._handleRAF = function () {
        (a._timerId = null), a._setupTick(), a._tick();
      }),
      (a._handleTimeout = function () {
        (a._timerId = null), a._setupTick(), a._tick();
      }),
      (a._setupTick = function () {
        if (null == a._timerId) {
          var b = a.timingMode || (a.useRAF && a.RAF_SYNCHED);
          if (b == a.RAF_SYNCHED || b == a.RAF) {
            var c =
              window.requestAnimationFrame ||
              window.webkitRequestAnimationFrame ||
              window.mozRequestAnimationFrame ||
              window.oRequestAnimationFrame ||
              window.msRequestAnimationFrame;
            if (c)
              return (
                (a._timerId = c(b == a.RAF ? a._handleRAF : a._handleSynch)),
                (a._raf = !0),
                void 0
              );
          }
          (a._raf = !1),
            (a._timerId = setTimeout(a._handleTimeout, a._interval));
        }
      }),
      (a._tick = function () {
        var b = a._getTime() - a._startTime,
          c = b - a._lastTime,
          d = a._paused;
        if (
          (a._ticks++,
          d && (a._pausedTicks++, (a._pausedTime += c)),
          (a._lastTime = b),
          a.hasEventListener("tick"))
        ) {
          var e = new createjs.Event("tick"),
            f = a.maxDelta;
          (e.delta = f && c > f ? f : c),
            (e.paused = d),
            (e.time = b),
            (e.runTime = b - a._pausedTime),
            a.dispatchEvent(e);
        }
        for (
          a._tickTimes.unshift(a._getTime() - b);
          a._tickTimes.length > 100;

        )
          a._tickTimes.pop();
        for (a._times.unshift(b); a._times.length > 100; ) a._times.pop();
      });
    var b =
      window.performance &&
      (performance.now ||
        performance.mozNow ||
        performance.msNow ||
        performance.oNow ||
        performance.webkitNow);
    (a._getTime = function () {
      return (b && b.call(performance)) || new Date().getTime();
    }),
      (createjs.Ticker = a);
  })(),
  (this.createjs = this.createjs || {}),
  (function () {
    "use strict";
    var a = function (a, b, c, d, e, f, g, h, i, j) {
        this.initialize(a, b, c, d, e, f, g, h, i, j);
      },
      b = (a.prototype = new createjs.Event());
    (b.stageX = 0),
      (b.stageY = 0),
      (b.rawX = 0),
      (b.rawY = 0),
      (b.nativeEvent = null),
      (b.pointerID = 0),
      (b.primary = !1),
      (b.addEventListener = null),
      (b.removeEventListener = null),
      (b.removeAllEventListeners = null),
      (b.dispatchEvent = null),
      (b.hasEventListener = null),
      (b._listeners = null),
      createjs.EventDispatcher.initialize(b),
      (b._get_localX = function () {
        return this.currentTarget.globalToLocal(this.rawX, this.rawY).x;
      }),
      (b._get_localY = function () {
        return this.currentTarget.globalToLocal(this.rawX, this.rawY).y;
      });
    try {
      Object.defineProperties(b, {
        localX: { get: b._get_localX },
        localY: { get: b._get_localY },
      });
    } catch (c) {}
    (b.Event_initialize = b.initialize),
      (b.initialize = function (a, b, c, d, e, f, g, h, i, j) {
        this.Event_initialize(a, b, c),
          (this.stageX = d),
          (this.stageY = e),
          (this.nativeEvent = f),
          (this.pointerID = g),
          (this.primary = h),
          (this.rawX = null == i ? d : i),
          (this.rawY = null == j ? e : j);
      }),
      (b.clone = function () {
        return new a(
          this.type,
          this.bubbles,
          this.cancelable,
          this.stageX,
          this.stageY,
          this.target,
          this.nativeEvent,
          this.pointerID,
          this.primary,
          this.rawX,
          this.rawY
        );
      }),
      (b.toString = function () {
        return (
          "[MouseEvent (type=" +
          this.type +
          " stageX=" +
          this.stageX +
          " stageY=" +
          this.stageY +
          ")]"
        );
      }),
      (createjs.MouseEvent = a);
  })(),
  (this.createjs = this.createjs || {}),
  (function () {
    "use strict";
    var a = function (a, b, c, d, e, f) {
        this.initialize(a, b, c, d, e, f);
      },
      b = a.prototype;
    (a.identity = null),
      (a.DEG_TO_RAD = Math.PI / 180),
      (b.a = 1),
      (b.b = 0),
      (b.c = 0),
      (b.d = 1),
      (b.tx = 0),
      (b.ty = 0),
      (b.alpha = 1),
      (b.shadow = null),
      (b.compositeOperation = null),
      (b.initialize = function (a, b, c, d, e, f) {
        return (
          (this.a = null == a ? 1 : a),
          (this.b = b || 0),
          (this.c = c || 0),
          (this.d = null == d ? 1 : d),
          (this.tx = e || 0),
          (this.ty = f || 0),
          this
        );
      }),
      (b.prepend = function (a, b, c, d, e, f) {
        var g = this.tx;
        if (1 != a || 0 != b || 0 != c || 1 != d) {
          var h = this.a,
            i = this.c;
          (this.a = h * a + this.b * c),
            (this.b = h * b + this.b * d),
            (this.c = i * a + this.d * c),
            (this.d = i * b + this.d * d);
        }
        return (
          (this.tx = g * a + this.ty * c + e),
          (this.ty = g * b + this.ty * d + f),
          this
        );
      }),
      (b.append = function (a, b, c, d, e, f) {
        var g = this.a,
          h = this.b,
          i = this.c,
          j = this.d;
        return (
          (this.a = a * g + b * i),
          (this.b = a * h + b * j),
          (this.c = c * g + d * i),
          (this.d = c * h + d * j),
          (this.tx = e * g + f * i + this.tx),
          (this.ty = e * h + f * j + this.ty),
          this
        );
      }),
      (b.prependMatrix = function (a) {
        return (
          this.prepend(a.a, a.b, a.c, a.d, a.tx, a.ty),
          this.prependProperties(a.alpha, a.shadow, a.compositeOperation),
          this
        );
      }),
      (b.appendMatrix = function (a) {
        return (
          this.append(a.a, a.b, a.c, a.d, a.tx, a.ty),
          this.appendProperties(a.alpha, a.shadow, a.compositeOperation),
          this
        );
      }),
      (b.prependTransform = function (b, c, d, e, f, g, h, i, j) {
        if (f % 360)
          var k = f * a.DEG_TO_RAD,
            l = Math.cos(k),
            m = Math.sin(k);
        else (l = 1), (m = 0);
        return (
          (i || j) && ((this.tx -= i), (this.ty -= j)),
          g || h
            ? ((g *= a.DEG_TO_RAD),
              (h *= a.DEG_TO_RAD),
              this.prepend(l * d, m * d, -m * e, l * e, 0, 0),
              this.prepend(
                Math.cos(h),
                Math.sin(h),
                -Math.sin(g),
                Math.cos(g),
                b,
                c
              ))
            : this.prepend(l * d, m * d, -m * e, l * e, b, c),
          this
        );
      }),
      (b.appendTransform = function (b, c, d, e, f, g, h, i, j) {
        if (f % 360)
          var k = f * a.DEG_TO_RAD,
            l = Math.cos(k),
            m = Math.sin(k);
        else (l = 1), (m = 0);
        return (
          g || h
            ? ((g *= a.DEG_TO_RAD),
              (h *= a.DEG_TO_RAD),
              this.append(
                Math.cos(h),
                Math.sin(h),
                -Math.sin(g),
                Math.cos(g),
                b,
                c
              ),
              this.append(l * d, m * d, -m * e, l * e, 0, 0))
            : this.append(l * d, m * d, -m * e, l * e, b, c),
          (i || j) &&
            ((this.tx -= i * this.a + j * this.c),
            (this.ty -= i * this.b + j * this.d)),
          this
        );
      }),
      (b.rotate = function (a) {
        var b = Math.cos(a),
          c = Math.sin(a),
          d = this.a,
          e = this.c,
          f = this.tx;
        return (
          (this.a = d * b - this.b * c),
          (this.b = d * c + this.b * b),
          (this.c = e * b - this.d * c),
          (this.d = e * c + this.d * b),
          (this.tx = f * b - this.ty * c),
          (this.ty = f * c + this.ty * b),
          this
        );
      }),
      (b.skew = function (b, c) {
        return (
          (b *= a.DEG_TO_RAD),
          (c *= a.DEG_TO_RAD),
          this.append(
            Math.cos(c),
            Math.sin(c),
            -Math.sin(b),
            Math.cos(b),
            0,
            0
          ),
          this
        );
      }),
      (b.scale = function (a, b) {
        return (
          (this.a *= a),
          (this.d *= b),
          (this.c *= a),
          (this.b *= b),
          (this.tx *= a),
          (this.ty *= b),
          this
        );
      }),
      (b.translate = function (a, b) {
        return (this.tx += a), (this.ty += b), this;
      }),
      (b.identity = function () {
        return (
          (this.alpha = this.a = this.d = 1),
          (this.b = this.c = this.tx = this.ty = 0),
          (this.shadow = this.compositeOperation = null),
          this
        );
      }),
      (b.invert = function () {
        var a = this.a,
          b = this.b,
          c = this.c,
          d = this.d,
          e = this.tx,
          f = a * d - b * c;
        return (
          (this.a = d / f),
          (this.b = -b / f),
          (this.c = -c / f),
          (this.d = a / f),
          (this.tx = (c * this.ty - d * e) / f),
          (this.ty = -(a * this.ty - b * e) / f),
          this
        );
      }),
      (b.isIdentity = function () {
        return (
          0 == this.tx &&
          0 == this.ty &&
          1 == this.a &&
          0 == this.b &&
          0 == this.c &&
          1 == this.d
        );
      }),
      (b.transformPoint = function (a, b, c) {
        return (
          (c = c || {}),
          (c.x = a * this.a + b * this.c + this.tx),
          (c.y = a * this.b + b * this.d + this.ty),
          c
        );
      }),
      (b.decompose = function (b) {
        null == b && (b = {}),
          (b.x = this.tx),
          (b.y = this.ty),
          (b.scaleX = Math.sqrt(this.a * this.a + this.b * this.b)),
          (b.scaleY = Math.sqrt(this.c * this.c + this.d * this.d));
        var c = Math.atan2(-this.c, this.d),
          d = Math.atan2(this.b, this.a);
        return (
          c == d
            ? ((b.rotation = d / a.DEG_TO_RAD),
              this.a < 0 &&
                this.d >= 0 &&
                (b.rotation += b.rotation <= 0 ? 180 : -180),
              (b.skewX = b.skewY = 0))
            : ((b.skewX = c / a.DEG_TO_RAD), (b.skewY = d / a.DEG_TO_RAD)),
          b
        );
      }),
      (b.reinitialize = function (a, b, c, d, e, f, g, h, i) {
        return (
          this.initialize(a, b, c, d, e, f),
          (this.alpha = null == g ? 1 : g),
          (this.shadow = h),
          (this.compositeOperation = i),
          this
        );
      }),
      (b.copy = function (a) {
        return this.reinitialize(
          a.a,
          a.b,
          a.c,
          a.d,
          a.tx,
          a.ty,
          a.alpha,
          a.shadow,
          a.compositeOperation
        );
      }),
      (b.appendProperties = function (a, b, c) {
        return (
          (this.alpha *= a),
          (this.shadow = b || this.shadow),
          (this.compositeOperation = c || this.compositeOperation),
          this
        );
      }),
      (b.prependProperties = function (a, b, c) {
        return (
          (this.alpha *= a),
          (this.shadow = this.shadow || b),
          (this.compositeOperation = this.compositeOperation || c),
          this
        );
      }),
      (b.clone = function () {
        return new a().copy(this);
      }),
      (b.toString = function () {
        return (
          "[Matrix2D (a=" +
          this.a +
          " b=" +
          this.b +
          " c=" +
          this.c +
          " d=" +
          this.d +
          " tx=" +
          this.tx +
          " ty=" +
          this.ty +
          ")]"
        );
      }),
      (a.identity = new a()),
      (createjs.Matrix2D = a);
  })(),
  (this.createjs = this.createjs || {}),
  (function () {
    "use strict";
    var a = function (a, b) {
        this.initialize(a, b);
      },
      b = a.prototype;
    (b.x = 0),
      (b.y = 0),
      (b.initialize = function (a, b) {
        return (this.x = null == a ? 0 : a), (this.y = null == b ? 0 : b), this;
      }),
      (b.copy = function (a) {
        return this.initialize(a.x, a.y);
      }),
      (b.clone = function () {
        return new a(this.x, this.y);
      }),
      (b.toString = function () {
        return "[Point (x=" + this.x + " y=" + this.y + ")]";
      }),
      (createjs.Point = a);
  })(),
  (this.createjs = this.createjs || {}),
  (function () {
    "use strict";
    var a = function (a, b, c, d) {
        this.initialize(a, b, c, d);
      },
      b = a.prototype;
    (b.x = 0),
      (b.y = 0),
      (b.width = 0),
      (b.height = 0),
      (b.initialize = function (a, b, c, d) {
        return (
          (this.x = a || 0),
          (this.y = b || 0),
          (this.width = c || 0),
          (this.height = d || 0),
          this
        );
      }),
      (b.copy = function (a) {
        return this.initialize(a.x, a.y, a.width, a.height);
      }),
      (b.clone = function () {
        return new a(this.x, this.y, this.width, this.height);
      }),
      (b.toString = function () {
        return (
          "[Rectangle (x=" +
          this.x +
          " y=" +
          this.y +
          " width=" +
          this.width +
          " height=" +
          this.height +
          ")]"
        );
      }),
      (createjs.Rectangle = a);
  })(),
  (this.createjs = this.createjs || {}),
  (function () {
    "use strict";
    var a = function (a, b, c, d, e, f, g) {
        this.initialize(a, b, c, d, e, f, g);
      },
      b = a.prototype;
    (b.target = null),
      (b.overLabel = null),
      (b.outLabel = null),
      (b.downLabel = null),
      (b.play = !1),
      (b._isPressed = !1),
      (b._isOver = !1),
      (b.initialize = function (a, b, c, d, e, f, g) {
        a.addEventListener &&
          ((this.target = a),
          (a.cursor = "pointer"),
          (this.overLabel = null == c ? "over" : c),
          (this.outLabel = null == b ? "out" : b),
          (this.downLabel = null == d ? "down" : d),
          (this.play = e),
          this.setEnabled(!0),
          this.handleEvent({}),
          f &&
            (g && ((f.actionsEnabled = !1), f.gotoAndStop && f.gotoAndStop(g)),
            (a.hitArea = f)));
      }),
      (b.setEnabled = function (a) {
        var b = this.target;
        a
          ? (b.addEventListener("rollover", this),
            b.addEventListener("rollout", this),
            b.addEventListener("mousedown", this),
            b.addEventListener("pressup", this))
          : (b.removeEventListener("rollover", this),
            b.removeEventListener("rollout", this),
            b.removeEventListener("mousedown", this),
            b.removeEventListener("pressup", this));
      }),
      (b.toString = function () {
        return "[ButtonHelper]";
      }),
      (b.handleEvent = function (a) {
        var b,
          c = this.target,
          d = a.type;
        "mousedown" == d
          ? ((this._isPressed = !0), (b = this.downLabel))
          : "pressup" == d
          ? ((this._isPressed = !1),
            (b = this._isOver ? this.overLabel : this.outLabel))
          : "rollover" == d
          ? ((this._isOver = !0),
            (b = this._isPressed ? this.downLabel : this.overLabel))
          : ((this._isOver = !1),
            (b = this._isPressed ? this.overLabel : this.outLabel)),
          this.play
            ? c.gotoAndPlay && c.gotoAndPlay(b)
            : c.gotoAndStop && c.gotoAndStop(b);
      }),
      (createjs.ButtonHelper = a);
  })(),
  (this.createjs = this.createjs || {}),
  (function () {
    "use strict";
    var a = function (a, b, c, d) {
        this.initialize(a, b, c, d);
      },
      b = a.prototype;
    (a.identity = null),
      (b.color = null),
      (b.offsetX = 0),
      (b.offsetY = 0),
      (b.blur = 0),
      (b.initialize = function (a, b, c, d) {
        (this.color = a),
          (this.offsetX = b),
          (this.offsetY = c),
          (this.blur = d);
      }),
      (b.toString = function () {
        return "[Shadow]";
      }),
      (b.clone = function () {
        return new a(this.color, this.offsetX, this.offsetY, this.blur);
      }),
      (a.identity = new a("transparent", 0, 0, 0)),
      (createjs.Shadow = a);
  })(),
  (this.createjs = this.createjs || {}),
  (function () {
    "use strict";
    var a = function (a) {
        this.initialize(a);
      },
      b = (a.prototype = new createjs.EventDispatcher());
    (b.complete = !0),
      (b.framerate = 0),
      (b._animations = null),
      (b._frames = null),
      (b._images = null),
      (b._data = null),
      (b._loadCount = 0),
      (b._frameHeight = 0),
      (b._frameWidth = 0),
      (b._numFrames = 0),
      (b._regX = 0),
      (b._regY = 0),
      (b.initialize = function (a) {
        var b, c, d, e;
        if (null != a) {
          if (
            ((this.framerate = a.framerate || 0),
            a.images && (c = a.images.length) > 0)
          )
            for (e = this._images = [], b = 0; c > b; b++) {
              var f = a.images[b];
              if ("string" == typeof f) {
                var g = f;
                (f = document.createElement("img")), (f.src = g);
              }
              e.push(f),
                f.getContext ||
                  f.complete ||
                  (this._loadCount++,
                  (this.complete = !1),
                  (function (a) {
                    f.onload = function () {
                      a._handleImageLoad();
                    };
                  })(this));
            }
          if (null == a.frames);
          else if (a.frames instanceof Array)
            for (
              this._frames = [], e = a.frames, b = 0, c = e.length;
              c > b;
              b++
            ) {
              var h = e[b];
              this._frames.push({
                image: this._images[h[4] ? h[4] : 0],
                rect: new createjs.Rectangle(h[0], h[1], h[2], h[3]),
                regX: h[5] || 0,
                regY: h[6] || 0,
              });
            }
          else
            (d = a.frames),
              (this._frameWidth = d.width),
              (this._frameHeight = d.height),
              (this._regX = d.regX || 0),
              (this._regY = d.regY || 0),
              (this._numFrames = d.count),
              0 == this._loadCount && this._calculateFrames();
          if (((this._animations = []), null != (d = a.animations))) {
            this._data = {};
            var i;
            for (i in d) {
              var j = { name: i },
                k = d[i];
              if ("number" == typeof k) e = j.frames = [k];
              else if (k instanceof Array)
                if (1 == k.length) j.frames = [k[0]];
                else
                  for (
                    j.speed = k[3], j.next = k[2], e = j.frames = [], b = k[0];
                    b <= k[1];
                    b++
                  )
                    e.push(b);
              else {
                (j.speed = k.speed), (j.next = k.next);
                var l = k.frames;
                e = j.frames = "number" == typeof l ? [l] : l.slice(0);
              }
              (j.next === !0 || void 0 === j.next) && (j.next = i),
                (j.next === !1 || (e.length < 2 && j.next == i)) &&
                  (j.next = null),
                j.speed || (j.speed = 1),
                this._animations.push(i),
                (this._data[i] = j);
            }
          }
        }
      }),
      (b.getNumFrames = function (a) {
        if (null == a)
          return this._frames ? this._frames.length : this._numFrames;
        var b = this._data[a];
        return null == b ? 0 : b.frames.length;
      }),
      (b.getAnimations = function () {
        return this._animations.slice(0);
      }),
      (b.getAnimation = function (a) {
        return this._data[a];
      }),
      (b.getFrame = function (a) {
        var b;
        return this._frames && (b = this._frames[a]) ? b : null;
      }),
      (b.getFrameBounds = function (a, b) {
        var c = this.getFrame(a);
        return c
          ? (b || new createjs.Rectangle()).initialize(
              -c.regX,
              -c.regY,
              c.rect.width,
              c.rect.height
            )
          : null;
      }),
      (b.toString = function () {
        return "[SpriteSheet]";
      }),
      (b.clone = function () {
        var b = new a();
        return (
          (b.complete = this.complete),
          (b._animations = this._animations),
          (b._frames = this._frames),
          (b._images = this._images),
          (b._data = this._data),
          (b._frameHeight = this._frameHeight),
          (b._frameWidth = this._frameWidth),
          (b._numFrames = this._numFrames),
          (b._loadCount = this._loadCount),
          b
        );
      }),
      (b._handleImageLoad = function () {
        0 == --this._loadCount &&
          (this._calculateFrames(),
          (this.complete = !0),
          this.dispatchEvent("complete"));
      }),
      (b._calculateFrames = function () {
        if (!this._frames && 0 != this._frameWidth) {
          this._frames = [];
          for (
            var a = 0,
              b = this._frameWidth,
              c = this._frameHeight,
              d = 0,
              e = this._images;
            d < e.length;
            d++
          ) {
            for (
              var f = e[d],
                g = 0 | (f.width / b),
                h = 0 | (f.height / c),
                i =
                  this._numFrames > 0
                    ? Math.min(this._numFrames - a, g * h)
                    : g * h,
                j = 0;
              i > j;
              j++
            )
              this._frames.push({
                image: f,
                rect: new createjs.Rectangle(
                  (j % g) * b,
                  (0 | (j / g)) * c,
                  b,
                  c
                ),
                regX: this._regX,
                regY: this._regY,
              });
            a += i;
          }
          this._numFrames = a;
        }
      }),
      (createjs.SpriteSheet = a);
  })(),
  (this.createjs = this.createjs || {}),
  (function () {
    "use strict";
    function a(a, b, c) {
      (this.f = a), (this.params = b), (this.path = null == c ? !0 : c);
    }
    a.prototype.exec = function (a) {
      this.f.apply(a, this.params);
    };
    var b = function () {
        this.initialize();
      },
      c = b.prototype;
    (b.getRGB = function (a, b, c, d) {
      return (
        null != a &&
          null == c &&
          ((d = b), (c = 255 & a), (b = 255 & (a >> 8)), (a = 255 & (a >> 16))),
        null == d
          ? "rgb(" + a + "," + b + "," + c + ")"
          : "rgba(" + a + "," + b + "," + c + "," + d + ")"
      );
    }),
      (b.getHSL = function (a, b, c, d) {
        return null == d
          ? "hsl(" + (a % 360) + "," + b + "%," + c + "%)"
          : "hsla(" + (a % 360) + "," + b + "%," + c + "%," + d + ")";
      }),
      (b.Command = a),
      (b.BASE_64 = {
        A: 0,
        B: 1,
        C: 2,
        D: 3,
        E: 4,
        F: 5,
        G: 6,
        H: 7,
        I: 8,
        J: 9,
        K: 10,
        L: 11,
        M: 12,
        N: 13,
        O: 14,
        P: 15,
        Q: 16,
        R: 17,
        S: 18,
        T: 19,
        U: 20,
        V: 21,
        W: 22,
        X: 23,
        Y: 24,
        Z: 25,
        a: 26,
        b: 27,
        c: 28,
        d: 29,
        e: 30,
        f: 31,
        g: 32,
        h: 33,
        i: 34,
        j: 35,
        k: 36,
        l: 37,
        m: 38,
        n: 39,
        o: 40,
        p: 41,
        q: 42,
        r: 43,
        s: 44,
        t: 45,
        u: 46,
        v: 47,
        w: 48,
        x: 49,
        y: 50,
        z: 51,
        0: 52,
        1: 53,
        2: 54,
        3: 55,
        4: 56,
        5: 57,
        6: 58,
        7: 59,
        8: 60,
        9: 61,
        "+": 62,
        "/": 63,
      }),
      (b.STROKE_CAPS_MAP = ["butt", "round", "square"]),
      (b.STROKE_JOINTS_MAP = ["miter", "round", "bevel"]);
    var d = createjs.createCanvas
      ? createjs.createCanvas()
      : document.createElement("canvas");
    if (d.getContext) {
      var e = (b._ctx = d.getContext("2d"));
      (b.beginCmd = new a(e.beginPath, [], !1)),
        (b.fillCmd = new a(e.fill, [], !1)),
        (b.strokeCmd = new a(e.stroke, [], !1)),
        (d.width = d.height = 1);
    }
    (c._strokeInstructions = null),
      (c._strokeStyleInstructions = null),
      (c._strokeIgnoreScale = !1),
      (c._fillInstructions = null),
      (c._fillMatrix = null),
      (c._instructions = null),
      (c._oldInstructions = null),
      (c._activeInstructions = null),
      (c._active = !1),
      (c._dirty = !1),
      (c.initialize = function () {
        this.clear(), (this._ctx = b._ctx);
      }),
      (c.isEmpty = function () {
        return !(
          this._instructions.length ||
          this._oldInstructions.length ||
          this._activeInstructions.length
        );
      }),
      (c.draw = function (a) {
        this._dirty && this._updateInstructions();
        for (var b = this._instructions, c = 0, d = b.length; d > c; c++)
          b[c].exec(a);
      }),
      (c.drawAsPath = function (a) {
        this._dirty && this._updateInstructions();
        for (var b, c = this._instructions, d = 0, e = c.length; e > d; d++)
          ((b = c[d]).path || 0 == d) && b.exec(a);
      }),
      (c.moveTo = function (b, c) {
        return (
          this._activeInstructions.push(new a(this._ctx.moveTo, [b, c])), this
        );
      }),
      (c.lineTo = function (b, c) {
        return (
          (this._dirty = this._active = !0),
          this._activeInstructions.push(new a(this._ctx.lineTo, [b, c])),
          this
        );
      }),
      (c.arcTo = function (b, c, d, e, f) {
        return (
          (this._dirty = this._active = !0),
          this._activeInstructions.push(
            new a(this._ctx.arcTo, [b, c, d, e, f])
          ),
          this
        );
      }),
      (c.arc = function (b, c, d, e, f, g) {
        return (
          (this._dirty = this._active = !0),
          null == g && (g = !1),
          this._activeInstructions.push(
            new a(this._ctx.arc, [b, c, d, e, f, g])
          ),
          this
        );
      }),
      (c.quadraticCurveTo = function (b, c, d, e) {
        return (
          (this._dirty = this._active = !0),
          this._activeInstructions.push(
            new a(this._ctx.quadraticCurveTo, [b, c, d, e])
          ),
          this
        );
      }),
      (c.bezierCurveTo = function (b, c, d, e, f, g) {
        return (
          (this._dirty = this._active = !0),
          this._activeInstructions.push(
            new a(this._ctx.bezierCurveTo, [b, c, d, e, f, g])
          ),
          this
        );
      }),
      (c.rect = function (b, c, d, e) {
        return (
          (this._dirty = this._active = !0),
          this._activeInstructions.push(new a(this._ctx.rect, [b, c, d, e])),
          this
        );
      }),
      (c.closePath = function () {
        return (
          this._active &&
            ((this._dirty = !0),
            this._activeInstructions.push(new a(this._ctx.closePath, []))),
          this
        );
      }),
      (c.clear = function () {
        return (
          (this._instructions = []),
          (this._oldInstructions = []),
          (this._activeInstructions = []),
          (this._strokeStyleInstructions = this._strokeInstructions = this._fillInstructions = this._fillMatrix = null),
          (this._active = this._dirty = this._strokeIgnoreScale = !1),
          this
        );
      }),
      (c.beginFill = function (b) {
        return (
          this._active && this._newPath(),
          (this._fillInstructions = b
            ? [new a(this._setProp, ["fillStyle", b], !1)]
            : null),
          (this._fillMatrix = null),
          this
        );
      }),
      (c.beginLinearGradientFill = function (b, c, d, e, f, g) {
        this._active && this._newPath();
        for (
          var h = this._ctx.createLinearGradient(d, e, f, g),
            i = 0,
            j = b.length;
          j > i;
          i++
        )
          h.addColorStop(c[i], b[i]);
        return (
          (this._fillInstructions = [
            new a(this._setProp, ["fillStyle", h], !1),
          ]),
          (this._fillMatrix = null),
          this
        );
      }),
      (c.beginRadialGradientFill = function (b, c, d, e, f, g, h, i) {
        this._active && this._newPath();
        for (
          var j = this._ctx.createRadialGradient(d, e, f, g, h, i),
            k = 0,
            l = b.length;
          l > k;
          k++
        )
          j.addColorStop(c[k], b[k]);
        return (
          (this._fillInstructions = [
            new a(this._setProp, ["fillStyle", j], !1),
          ]),
          (this._fillMatrix = null),
          this
        );
      }),
      (c.beginBitmapFill = function (b, c, d) {
        this._active && this._newPath(), (c = c || "");
        var e = this._ctx.createPattern(b, c);
        return (
          (this._fillInstructions = [
            new a(this._setProp, ["fillStyle", e], !1),
          ]),
          (this._fillMatrix = d ? [d.a, d.b, d.c, d.d, d.tx, d.ty] : null),
          this
        );
      }),
      (c.endFill = function () {
        return this.beginFill();
      }),
      (c.setStrokeStyle = function (c, d, e, f, g) {
        return (
          this._active && this._newPath(),
          (this._strokeStyleInstructions = [
            new a(this._setProp, ["lineWidth", null == c ? "1" : c], !1),
            new a(
              this._setProp,
              [
                "lineCap",
                null == d ? "butt" : isNaN(d) ? d : b.STROKE_CAPS_MAP[d],
              ],
              !1
            ),
            new a(
              this._setProp,
              [
                "lineJoin",
                null == e ? "miter" : isNaN(e) ? e : b.STROKE_JOINTS_MAP[e],
              ],
              !1
            ),
            new a(this._setProp, ["miterLimit", null == f ? "10" : f], !1),
          ]),
          (this._strokeIgnoreScale = g),
          this
        );
      }),
      (c.beginStroke = function (b) {
        return (
          this._active && this._newPath(),
          (this._strokeInstructions = b
            ? [new a(this._setProp, ["strokeStyle", b], !1)]
            : null),
          this
        );
      }),
      (c.beginLinearGradientStroke = function (b, c, d, e, f, g) {
        this._active && this._newPath();
        for (
          var h = this._ctx.createLinearGradient(d, e, f, g),
            i = 0,
            j = b.length;
          j > i;
          i++
        )
          h.addColorStop(c[i], b[i]);
        return (
          (this._strokeInstructions = [
            new a(this._setProp, ["strokeStyle", h], !1),
          ]),
          this
        );
      }),
      (c.beginRadialGradientStroke = function (b, c, d, e, f, g, h, i) {
        this._active && this._newPath();
        for (
          var j = this._ctx.createRadialGradient(d, e, f, g, h, i),
            k = 0,
            l = b.length;
          l > k;
          k++
        )
          j.addColorStop(c[k], b[k]);
        return (
          (this._strokeInstructions = [
            new a(this._setProp, ["strokeStyle", j], !1),
          ]),
          this
        );
      }),
      (c.beginBitmapStroke = function (b, c) {
        this._active && this._newPath(), (c = c || "");
        var d = this._ctx.createPattern(b, c);
        return (
          (this._strokeInstructions = [
            new a(this._setProp, ["strokeStyle", d], !1),
          ]),
          this
        );
      }),
      (c.endStroke = function () {
        return this.beginStroke(), this;
      }),
      (c.curveTo = c.quadraticCurveTo),
      (c.drawRect = c.rect),
      (c.drawRoundRect = function (a, b, c, d, e) {
        return this.drawRoundRectComplex(a, b, c, d, e, e, e, e), this;
      }),
      (c.drawRoundRectComplex = function (b, c, d, e, f, g, h, i) {
        var j = (e > d ? d : e) / 2,
          k = 0,
          l = 0,
          m = 0,
          n = 0;
        0 > f && (f *= k = -1),
          f > j && (f = j),
          0 > g && (g *= l = -1),
          g > j && (g = j),
          0 > h && (h *= m = -1),
          h > j && (h = j),
          0 > i && (i *= n = -1),
          i > j && (i = j),
          (this._dirty = this._active = !0);
        var o = this._ctx.arcTo,
          p = this._ctx.lineTo;
        return (
          this._activeInstructions.push(
            new a(this._ctx.moveTo, [b + d - g, c]),
            new a(o, [b + d + g * l, c - g * l, b + d, c + g, g]),
            new a(p, [b + d, c + e - h]),
            new a(o, [b + d + h * m, c + e + h * m, b + d - h, c + e, h]),
            new a(p, [b + i, c + e]),
            new a(o, [b - i * n, c + e + i * n, b, c + e - i, i]),
            new a(p, [b, c + f]),
            new a(o, [b - f * k, c - f * k, b + f, c, f]),
            new a(this._ctx.closePath)
          ),
          this
        );
      }),
      (c.drawCircle = function (a, b, c) {
        return this.arc(a, b, c, 0, 2 * Math.PI), this;
      }),
      (c.drawEllipse = function (b, c, d, e) {
        this._dirty = this._active = !0;
        var f = 0.5522848,
          g = (d / 2) * f,
          h = (e / 2) * f,
          i = b + d,
          j = c + e,
          k = b + d / 2,
          l = c + e / 2;
        return (
          this._activeInstructions.push(
            new a(this._ctx.moveTo, [b, l]),
            new a(this._ctx.bezierCurveTo, [b, l - h, k - g, c, k, c]),
            new a(this._ctx.bezierCurveTo, [k + g, c, i, l - h, i, l]),
            new a(this._ctx.bezierCurveTo, [i, l + h, k + g, j, k, j]),
            new a(this._ctx.bezierCurveTo, [k - g, j, b, l + h, b, l])
          ),
          this
        );
      }),
      (c.inject = function (b, c) {
        return (
          (this._dirty = this._active = !0),
          this._activeInstructions.push(new a(b, [c])),
          this
        );
      }),
      (c.drawPolyStar = function (b, c, d, e, f, g) {
        (this._dirty = this._active = !0),
          null == f && (f = 0),
          (f = 1 - f),
          null == g ? (g = 0) : (g /= 180 / Math.PI);
        var h = Math.PI / e;
        this._activeInstructions.push(
          new a(this._ctx.moveTo, [b + Math.cos(g) * d, c + Math.sin(g) * d])
        );
        for (var i = 0; e > i; i++)
          (g += h),
            1 != f &&
              this._activeInstructions.push(
                new a(this._ctx.lineTo, [
                  b + Math.cos(g) * d * f,
                  c + Math.sin(g) * d * f,
                ])
              ),
            (g += h),
            this._activeInstructions.push(
              new a(this._ctx.lineTo, [
                b + Math.cos(g) * d,
                c + Math.sin(g) * d,
              ])
            );
        return this;
      }),
      (c.decodePath = function (a) {
        for (
          var c = [
              this.moveTo,
              this.lineTo,
              this.quadraticCurveTo,
              this.bezierCurveTo,
              this.closePath,
            ],
            d = [2, 2, 4, 6, 0],
            e = 0,
            f = a.length,
            g = [],
            h = 0,
            i = 0,
            j = b.BASE_64;
          f > e;

        ) {
          var k = a.charAt(e),
            l = j[k],
            m = l >> 3,
            n = c[m];
          if (!n || 3 & l) throw "bad path data (@" + e + "): " + k;
          var o = d[m];
          m || (h = i = 0), (g.length = 0), e++;
          for (var p = (1 & (l >> 2)) + 2, q = 0; o > q; q++) {
            var r = j[a.charAt(e)],
              s = r >> 5 ? -1 : 1;
            (r = ((31 & r) << 6) | j[a.charAt(e + 1)]),
              3 == p && (r = (r << 6) | j[a.charAt(e + 2)]),
              (r = (s * r) / 10),
              q % 2 ? (h = r += h) : (i = r += i),
              (g[q] = r),
              (e += p);
          }
          n.apply(this, g);
        }
        return this;
      }),
      (c.clone = function () {
        var a = new b();
        return (
          (a._instructions = this._instructions.slice()),
          (a._activeInstructions = this._activeInstructions.slice()),
          (a._oldInstructions = this._oldInstructions.slice()),
          this._fillInstructions &&
            (a._fillInstructions = this._fillInstructions.slice()),
          this._strokeInstructions &&
            (a._strokeInstructions = this._strokeInstructions.slice()),
          this._strokeStyleInstructions &&
            (a._strokeStyleInstructions = this._strokeStyleInstructions.slice()),
          (a._active = this._active),
          (a._dirty = this._dirty),
          (a._fillMatrix = this._fillMatrix),
          (a._strokeIgnoreScale = this._strokeIgnoreScale),
          a
        );
      }),
      (c.toString = function () {
        return "[Graphics]";
      }),
      (c.mt = c.moveTo),
      (c.lt = c.lineTo),
      (c.at = c.arcTo),
      (c.bt = c.bezierCurveTo),
      (c.qt = c.quadraticCurveTo),
      (c.a = c.arc),
      (c.r = c.rect),
      (c.cp = c.closePath),
      (c.c = c.clear),
      (c.f = c.beginFill),
      (c.lf = c.beginLinearGradientFill),
      (c.rf = c.beginRadialGradientFill),
      (c.bf = c.beginBitmapFill),
      (c.ef = c.endFill),
      (c.ss = c.setStrokeStyle),
      (c.s = c.beginStroke),
      (c.ls = c.beginLinearGradientStroke),
      (c.rs = c.beginRadialGradientStroke),
      (c.bs = c.beginBitmapStroke),
      (c.es = c.endStroke),
      (c.dr = c.drawRect),
      (c.rr = c.drawRoundRect),
      (c.rc = c.drawRoundRectComplex),
      (c.dc = c.drawCircle),
      (c.de = c.drawEllipse),
      (c.dp = c.drawPolyStar),
      (c.p = c.decodePath),
      (c._updateInstructions = function () {
        (this._instructions = this._oldInstructions.slice()),
          this._instructions.push(b.beginCmd),
          this._appendInstructions(this._fillInstructions),
          this._appendInstructions(this._strokeInstructions),
          this._appendInstructions(
            this._strokeInstructions && this._strokeStyleInstructions
          ),
          this._appendInstructions(this._activeInstructions),
          this._fillInstructions &&
            this._appendDraw(b.fillCmd, this._fillMatrix),
          this._strokeInstructions &&
            this._appendDraw(
              b.strokeCmd,
              this._strokeIgnoreScale && [1, 0, 0, 1, 0, 0]
            );
      }),
      (c._appendInstructions = function (a) {
        a && this._instructions.push.apply(this._instructions, a);
      }),
      (c._appendDraw = function (b, c) {
        c
          ? this._instructions.push(
              new a(this._ctx.save, [], !1),
              new a(this._ctx.transform, c, !1),
              b,
              new a(this._ctx.restore, [], !1)
            )
          : this._instructions.push(b);
      }),
      (c._newPath = function () {
        this._dirty && this._updateInstructions(),
          (this._oldInstructions = this._instructions),
          (this._activeInstructions = []),
          (this._active = this._dirty = !1);
      }),
      (c._setProp = function (a, b) {
        this[a] = b;
      }),
      (createjs.Graphics = b);
  })(),
  (this.createjs = this.createjs || {}),
  (function () {
    var a = function () {
        this.initialize();
      },
      b = (a.prototype = new createjs.EventDispatcher());
    (a._MOUSE_EVENTS = [
      "click",
      "dblclick",
      "mousedown",
      "mouseout",
      "mouseover",
      "pressmove",
      "pressup",
      "rollout",
      "rollover",
    ]),
      (a.suppressCrossDomainErrors = !1);
    var c = createjs.createCanvas
      ? createjs.createCanvas()
      : document.createElement("canvas");
    c.getContext &&
      ((a._hitTestCanvas = c),
      (a._hitTestContext = c.getContext("2d")),
      (c.width = c.height = 1)),
      (a._nextCacheID = 1),
      (b.alpha = 1),
      (b.cacheCanvas = null),
      (b.id = -1),
      (b.mouseEnabled = !0),
      (b.tickEnabled = !0),
      (b.name = null),
      (b.parent = null),
      (b.regX = 0),
      (b.regY = 0),
      (b.rotation = 0),
      (b.scaleX = 1),
      (b.scaleY = 1),
      (b.skewX = 0),
      (b.skewY = 0),
      (b.shadow = null),
      (b.visible = !0),
      (b.x = 0),
      (b.y = 0),
      (b.compositeOperation = null),
      (b.snapToPixel = !1),
      (b.filters = null),
      (b.cacheID = 0),
      (b.mask = null),
      (b.hitArea = null),
      (b.cursor = null),
      (b._cacheOffsetX = 0),
      (b._cacheOffsetY = 0),
      (b._cacheScale = 1),
      (b._cacheDataURLID = 0),
      (b._cacheDataURL = null),
      (b._matrix = null),
      (b._rectangle = null),
      (b._bounds = null),
      (b.initialize = function () {
        (this.id = createjs.UID.get()),
          (this._matrix = new createjs.Matrix2D()),
          (this._rectangle = new createjs.Rectangle());
      }),
      (b.isVisible = function () {
        return !!(
          this.visible &&
          this.alpha > 0 &&
          0 != this.scaleX &&
          0 != this.scaleY
        );
      }),
      (b.draw = function (a, b) {
        var c = this.cacheCanvas;
        if (b || !c) return !1;
        var d,
          e = this._cacheScale,
          f = this._cacheOffsetX,
          g = this._cacheOffsetY;
        return (
          (d = this._applyFilterBounds(f, g, 0, 0)) && ((f = d.x), (g = d.y)),
          a.drawImage(c, f, g, c.width / e, c.height / e),
          !0
        );
      }),
      (b.updateContext = function (a) {
        var b,
          c = this.mask,
          d = this;
        c &&
          c.graphics &&
          !c.graphics.isEmpty() &&
          ((b = c.getMatrix(c._matrix)),
          a.transform(b.a, b.b, b.c, b.d, b.tx, b.ty),
          c.graphics.drawAsPath(a),
          a.clip(),
          b.invert(),
          a.transform(b.a, b.b, b.c, b.d, b.tx, b.ty)),
          (b = d._matrix
            .identity()
            .appendTransform(
              d.x,
              d.y,
              d.scaleX,
              d.scaleY,
              d.rotation,
              d.skewX,
              d.skewY,
              d.regX,
              d.regY
            )),
          createjs.Stage._snapToPixelEnabled && d.snapToPixel
            ? a.transform(
                b.a,
                b.b,
                b.c,
                b.d,
                0 | (b.tx + 0.5),
                0 | (b.ty + 0.5)
              )
            : a.transform(b.a, b.b, b.c, b.d, b.tx, b.ty),
          (a.globalAlpha *= d.alpha),
          d.compositeOperation &&
            (a.globalCompositeOperation = d.compositeOperation),
          d.shadow && this._applyShadow(a, d.shadow);
      }),
      (b.cache = function (a, b, c, d, e) {
        (e = e || 1),
          this.cacheCanvas ||
            (this.cacheCanvas = createjs.createCanvas
              ? createjs.createCanvas()
              : document.createElement("canvas")),
          (this._cacheWidth = c),
          (this._cacheHeight = d),
          (this._cacheOffsetX = a),
          (this._cacheOffsetY = b),
          (this._cacheScale = e),
          this.updateCache();
      }),
      (b.updateCache = function (b) {
        var c,
          d = this.cacheCanvas,
          e = this._cacheScale,
          f = this._cacheOffsetX * e,
          g = this._cacheOffsetY * e,
          h = this._cacheWidth,
          i = this._cacheHeight;
        if (!d) throw "cache() must be called before updateCache()";
        var j = d.getContext("2d");
        (c = this._applyFilterBounds(f, g, h, i)) &&
          ((f = c.x), (g = c.y), (h = c.width), (i = c.height)),
          (h = Math.ceil(h * e)),
          (i = Math.ceil(i * e)),
          h != d.width || i != d.height
            ? ((d.width = h), (d.height = i))
            : b || j.clearRect(0, 0, h + 1, i + 1),
          j.save(),
          (j.globalCompositeOperation = b),
          j.setTransform(e, 0, 0, e, -f, -g),
          this.draw(j, !0),
          this._applyFilters(),
          j.restore(),
          (this.cacheID = a._nextCacheID++);
      }),
      (b.uncache = function () {
        (this._cacheDataURL = this.cacheCanvas = null),
          (this.cacheID = this._cacheOffsetX = this._cacheOffsetY = 0),
          (this._cacheScale = 1);
      }),
      (b.getCacheDataURL = function () {
        return this.cacheCanvas
          ? (this.cacheID != this._cacheDataURLID &&
              (this._cacheDataURL = this.cacheCanvas.toDataURL()),
            this._cacheDataURL)
          : null;
      }),
      (b.getStage = function () {
        for (var a = this; a.parent; ) a = a.parent;
        return a instanceof createjs.Stage ? a : null;
      }),
      (b.localToGlobal = function (a, b) {
        var c = this.getConcatenatedMatrix(this._matrix);
        return null == c
          ? null
          : (c.append(1, 0, 0, 1, a, b), new createjs.Point(c.tx, c.ty));
      }),
      (b.globalToLocal = function (a, b) {
        var c = this.getConcatenatedMatrix(this._matrix);
        return null == c
          ? null
          : (c.invert(),
            c.append(1, 0, 0, 1, a, b),
            new createjs.Point(c.tx, c.ty));
      }),
      (b.localToLocal = function (a, b, c) {
        var d = this.localToGlobal(a, b);
        return c.globalToLocal(d.x, d.y);
      }),
      (b.setTransform = function (a, b, c, d, e, f, g, h, i) {
        return (
          (this.x = a || 0),
          (this.y = b || 0),
          (this.scaleX = null == c ? 1 : c),
          (this.scaleY = null == d ? 1 : d),
          (this.rotation = e || 0),
          (this.skewX = f || 0),
          (this.skewY = g || 0),
          (this.regX = h || 0),
          (this.regY = i || 0),
          this
        );
      }),
      (b.getMatrix = function (a) {
        var b = this;
        return (a ? a.identity() : new createjs.Matrix2D())
          .appendTransform(
            b.x,
            b.y,
            b.scaleX,
            b.scaleY,
            b.rotation,
            b.skewX,
            b.skewY,
            b.regX,
            b.regY
          )
          .appendProperties(b.alpha, b.shadow, b.compositeOperation);
      }),
      (b.getConcatenatedMatrix = function (a) {
        a ? a.identity() : (a = new createjs.Matrix2D());
        for (var b = this; null != b; )
          a
            .prependTransform(
              b.x,
              b.y,
              b.scaleX,
              b.scaleY,
              b.rotation,
              b.skewX,
              b.skewY,
              b.regX,
              b.regY
            )
            .prependProperties(b.alpha, b.shadow, b.compositeOperation),
            (b = b.parent);
        return a;
      }),
      (b.hitTest = function (b, c) {
        var d = a._hitTestContext;
        d.setTransform(1, 0, 0, 1, -b, -c), this.draw(d);
        var e = this._testHit(d);
        return d.setTransform(1, 0, 0, 1, 0, 0), d.clearRect(0, 0, 2, 2), e;
      }),
      (b.set = function (a) {
        for (var b in a) this[b] = a[b];
        return this;
      }),
      (b.getBounds = function () {
        if (this._bounds) return this._rectangle.copy(this._bounds);
        var a = this.cacheCanvas;
        if (a) {
          var b = this._cacheScale;
          return this._rectangle.initialize(
            this._cacheOffsetX,
            this._cacheOffsetY,
            a.width / b,
            a.height / b
          );
        }
        return null;
      }),
      (b.getTransformedBounds = function () {
        return this._getBounds();
      }),
      (b.setBounds = function (a, b, c, d) {
        null == a && (this._bounds = a),
          (this._bounds = (this._bounds || new createjs.Rectangle()).initialize(
            a,
            b,
            c,
            d
          ));
      }),
      (b.clone = function () {
        var b = new a();
        return this.cloneProps(b), b;
      }),
      (b.toString = function () {
        return "[DisplayObject (name=" + this.name + ")]";
      }),
      (b.cloneProps = function (a) {
        (a.alpha = this.alpha),
          (a.name = this.name),
          (a.regX = this.regX),
          (a.regY = this.regY),
          (a.rotation = this.rotation),
          (a.scaleX = this.scaleX),
          (a.scaleY = this.scaleY),
          (a.shadow = this.shadow),
          (a.skewX = this.skewX),
          (a.skewY = this.skewY),
          (a.visible = this.visible),
          (a.x = this.x),
          (a.y = this.y),
          (a._bounds = this._bounds),
          (a.mouseEnabled = this.mouseEnabled),
          (a.compositeOperation = this.compositeOperation);
      }),
      (b._applyShadow = function (a, b) {
        (b = b || Shadow.identity),
          (a.shadowColor = b.color),
          (a.shadowOffsetX = b.offsetX),
          (a.shadowOffsetY = b.offsetY),
          (a.shadowBlur = b.blur);
      }),
      (b._tick = function (a) {
        var b = this._listeners;
        if (b && b.tick) {
          var c = new createjs.Event("tick");
          (c.params = a), this._dispatchEvent(c, this, 2);
        }
      }),
      (b._testHit = function (b) {
        try {
          var c = b.getImageData(0, 0, 1, 1).data[3] > 1;
        } catch (d) {
          if (!a.suppressCrossDomainErrors)
            throw "An error has occurred. This is most likely due to security restrictions on reading canvas pixel data with local or cross-domain images.";
        }
        return c;
      }),
      (b._applyFilters = function () {
        if (this.filters && 0 != this.filters.length && this.cacheCanvas)
          for (
            var a = this.filters.length,
              b = this.cacheCanvas.getContext("2d"),
              c = this.cacheCanvas.width,
              d = this.cacheCanvas.height,
              e = 0;
            a > e;
            e++
          )
            this.filters[e].applyFilter(b, 0, 0, c, d);
      }),
      (b._applyFilterBounds = function (a, b, c, d) {
        var e,
          f,
          g = this.filters;
        if (!g || !(f = g.length)) return null;
        for (var h = 0; f > h; h++) {
          var i = this.filters[h],
            j = i.getBounds && i.getBounds();
          j &&
            (e || (e = this._rectangle.initialize(a, b, c, d)),
            (e.x += j.x),
            (e.y += j.y),
            (e.width += j.width),
            (e.height += j.height));
        }
        return e;
      }),
      (b._getBounds = function (a, b) {
        return this._transformBounds(this.getBounds(), a, b);
      }),
      (b._transformBounds = function (a, b, c) {
        if (!a) return a;
        var d = a.x,
          e = a.y,
          f = a.width,
          g = a.height,
          h = c ? this._matrix.identity() : this.getMatrix(this._matrix);
        (d || e) && h.appendTransform(0, 0, 1, 1, 0, 0, 0, -d, -e),
          b && h.prependMatrix(b);
        var i = f * h.a,
          j = f * h.b,
          k = g * h.c,
          l = g * h.d,
          m = h.tx,
          n = h.ty,
          o = m,
          p = m,
          q = n,
          r = n;
        return (
          (d = i + m) < o ? (o = d) : d > p && (p = d),
          (d = i + k + m) < o ? (o = d) : d > p && (p = d),
          (d = k + m) < o ? (o = d) : d > p && (p = d),
          (e = j + n) < q ? (q = e) : e > r && (r = e),
          (e = j + l + n) < q ? (q = e) : e > r && (r = e),
          (e = l + n) < q ? (q = e) : e > r && (r = e),
          a.initialize(o, q, p - o, r - q)
        );
      }),
      (b._hasMouseEventListener = function () {
        for (var b = a._MOUSE_EVENTS, c = 0, d = b.length; d > c; c++)
          if (this.hasEventListener(b[c])) return !0;
        return !!this.cursor;
      }),
      (createjs.DisplayObject = a);
  })(),
  (this.createjs = this.createjs || {}),
  (function () {
    var a = function () {
        this.initialize();
      },
      b = (a.prototype = new createjs.DisplayObject());
    (b.children = null),
      (b.mouseChildren = !0),
      (b.tickChildren = !0),
      (b.DisplayObject_initialize = b.initialize),
      (b.initialize = function () {
        this.DisplayObject_initialize(), (this.children = []);
      }),
      (b.isVisible = function () {
        var a = this.cacheCanvas || this.children.length;
        return !!(
          this.visible &&
          this.alpha > 0 &&
          0 != this.scaleX &&
          0 != this.scaleY &&
          a
        );
      }),
      (b.DisplayObject_draw = b.draw),
      (b.draw = function (a, b) {
        if (this.DisplayObject_draw(a, b)) return !0;
        for (var c = this.children.slice(0), d = 0, e = c.length; e > d; d++) {
          var f = c[d];
          f.isVisible() &&
            (a.save(), f.updateContext(a), f.draw(a), a.restore());
        }
        return !0;
      }),
      (b.addChild = function (a) {
        if (null == a) return a;
        var b = arguments.length;
        if (b > 1) {
          for (var c = 0; b > c; c++) this.addChild(arguments[c]);
          return arguments[b - 1];
        }
        return (
          a.parent && a.parent.removeChild(a),
          (a.parent = this),
          this.children.push(a),
          a
        );
      }),
      (b.addChildAt = function (a, b) {
        var c = arguments.length,
          d = arguments[c - 1];
        if (0 > d || d > this.children.length) return arguments[c - 2];
        if (c > 2) {
          for (var e = 0; c - 1 > e; e++) this.addChildAt(arguments[e], d + e);
          return arguments[c - 2];
        }
        return (
          a.parent && a.parent.removeChild(a),
          (a.parent = this),
          this.children.splice(b, 0, a),
          a
        );
      }),
      (b.removeChild = function (a) {
        var b = arguments.length;
        if (b > 1) {
          for (var c = !0, d = 0; b > d; d++)
            c = c && this.removeChild(arguments[d]);
          return c;
        }
        return this.removeChildAt(createjs.indexOf(this.children, a));
      }),
      (b.removeChildAt = function (a) {
        var b = arguments.length;
        if (b > 1) {
          for (var c = [], d = 0; b > d; d++) c[d] = arguments[d];
          c.sort(function (a, b) {
            return b - a;
          });
          for (var e = !0, d = 0; b > d; d++) e = e && this.removeChildAt(c[d]);
          return e;
        }
        if (0 > a || a > this.children.length - 1) return !1;
        var f = this.children[a];
        return f && (f.parent = null), this.children.splice(a, 1), !0;
      }),
      (b.removeAllChildren = function () {
        for (var a = this.children; a.length; ) a.pop().parent = null;
      }),
      (b.getChildAt = function (a) {
        return this.children[a];
      }),
      (b.getChildByName = function (a) {
        for (var b = this.children, c = 0, d = b.length; d > c; c++)
          if (b[c].name == a) return b[c];
        return null;
      }),
      (b.sortChildren = function (a) {
        this.children.sort(a);
      }),
      (b.getChildIndex = function (a) {
        return createjs.indexOf(this.children, a);
      }),
      (b.getNumChildren = function () {
        return this.children.length;
      }),
      (b.swapChildrenAt = function (a, b) {
        var c = this.children,
          d = c[a],
          e = c[b];
        d && e && ((c[a] = e), (c[b] = d));
      }),
      (b.swapChildren = function (a, b) {
        for (
          var c, d, e = this.children, f = 0, g = e.length;
          g > f &&
          (e[f] == a && (c = f), e[f] == b && (d = f), null == c || null == d);
          f++
        );
        f != g && ((e[c] = b), (e[d] = a));
      }),
      (b.setChildIndex = function (a, b) {
        var c = this.children,
          d = c.length;
        if (!(a.parent != this || 0 > b || b >= d)) {
          for (var e = 0; d > e && c[e] != a; e++);
          e != d && e != b && (c.splice(e, 1), c.splice(b, 0, a));
        }
      }),
      (b.contains = function (a) {
        for (; a; ) {
          if (a == this) return !0;
          a = a.parent;
        }
        return !1;
      }),
      (b.hitTest = function (a, b) {
        return null != this.getObjectUnderPoint(a, b);
      }),
      (b.getObjectsUnderPoint = function (a, b) {
        var c = [],
          d = this.localToGlobal(a, b);
        return this._getObjectsUnderPoint(d.x, d.y, c), c;
      }),
      (b.getObjectUnderPoint = function (a, b) {
        var c = this.localToGlobal(a, b);
        return this._getObjectsUnderPoint(c.x, c.y);
      }),
      (b.DisplayObject_getBounds = b.getBounds),
      (b.getBounds = function () {
        return this._getBounds(null, !0);
      }),
      (b.getTransformedBounds = function () {
        return this._getBounds();
      }),
      (b.clone = function (b) {
        var c = new a();
        if ((this.cloneProps(c), b))
          for (
            var d = (c.children = []), e = 0, f = this.children.length;
            f > e;
            e++
          ) {
            var g = this.children[e].clone(b);
            (g.parent = c), d.push(g);
          }
        return c;
      }),
      (b.toString = function () {
        return "[Container (name=" + this.name + ")]";
      }),
      (b.DisplayObject__tick = b._tick),
      (b._tick = function (a) {
        if (this.tickChildren)
          for (var b = this.children.length - 1; b >= 0; b--) {
            var c = this.children[b];
            c.tickEnabled && c._tick && c._tick(a);
          }
        this.DisplayObject__tick(a);
      }),
      (b._getObjectsUnderPoint = function (b, c, d, e, f) {
        var g = createjs.DisplayObject._hitTestContext,
          h = this._matrix;
        f = f || (e && this._hasMouseEventListener());
        for (var i = this.children, j = i.length, k = j - 1; k >= 0; k--) {
          var l = i[k],
            m = l.hitArea;
          if (l.visible && (m || l.isVisible()) && (!e || l.mouseEnabled))
            if (!m && l instanceof a) {
              var n = l._getObjectsUnderPoint(b, c, d, e, f);
              if (!d && n) return e && !this.mouseChildren ? this : n;
            } else {
              if (!f && !l._hasMouseEventListener()) continue;
              if (
                (l.getConcatenatedMatrix(h),
                m &&
                  (h.appendTransform(
                    m.x,
                    m.y,
                    m.scaleX,
                    m.scaleY,
                    m.rotation,
                    m.skewX,
                    m.skewY,
                    m.regX,
                    m.regY
                  ),
                  (h.alpha = m.alpha)),
                (g.globalAlpha = h.alpha),
                g.setTransform(h.a, h.b, h.c, h.d, h.tx - b, h.ty - c),
                (m || l).draw(g),
                !this._testHit(g))
              )
                continue;
              if (
                (g.setTransform(1, 0, 0, 1, 0, 0), g.clearRect(0, 0, 2, 2), !d)
              )
                return e && !this.mouseChildren ? this : l;
              d.push(l);
            }
        }
        return null;
      }),
      (b._getBounds = function (a, b) {
        var c = this.DisplayObject_getBounds();
        if (c) return this._transformBounds(c, a, b);
        var d,
          e,
          f,
          g,
          h = b ? this._matrix.identity() : this.getMatrix(this._matrix);
        a && h.prependMatrix(a);
        for (var i = this.children.length, j = 0; i > j; j++) {
          var k = this.children[j];
          if (k.visible && (c = k._getBounds(h))) {
            var l = c.x,
              m = c.y,
              n = l + c.width,
              o = m + c.height;
            (d > l || null == d) && (d = l),
              (n > e || null == e) && (e = n),
              (f > m || null == f) && (f = m),
              (o > g || null == g) && (g = o);
          }
        }
        return null == e
          ? null
          : this._rectangle.initialize(d, f, e - d, g - f);
      }),
      (createjs.Container = a);
  })(),
  (this.createjs = this.createjs || {}),
  (function () {
    "use strict";
    var a = function (a) {
        this.initialize(a);
      },
      b = (a.prototype = new createjs.Container());
    (a._snapToPixelEnabled = !1),
      (b.autoClear = !0),
      (b.canvas = null),
      (b.mouseX = 0),
      (b.mouseY = 0),
      (b.snapToPixelEnabled = !1),
      (b.mouseInBounds = !1),
      (b.tickOnUpdate = !0),
      (b.mouseMoveOutside = !1),
      (b.nextStage = null),
      (b._pointerData = null),
      (b._pointerCount = 0),
      (b._primaryPointerID = null),
      (b._mouseOverIntervalID = null),
      (b.Container_initialize = b.initialize),
      (b.initialize = function (a) {
        this.Container_initialize(),
          (this.canvas = "string" == typeof a ? document.getElementById(a) : a),
          (this._pointerData = {}),
          this.enableDOMEvents(!0);
      }),
      (b.update = function () {
        if (this.canvas) {
          this.tickOnUpdate &&
            (this.dispatchEvent("tickstart"),
            this.tickEnabled && this._tick(arguments.length ? arguments : null),
            this.dispatchEvent("tickend")),
            this.dispatchEvent("drawstart"),
            (a._snapToPixelEnabled = this.snapToPixelEnabled),
            this.autoClear && this.clear();
          var b = this.canvas.getContext("2d");
          b.save(),
            this.updateContext(b),
            this.draw(b, !1),
            b.restore(),
            this.dispatchEvent("drawend");
        }
      }),
      (b.handleEvent = function (a) {
        "tick" == a.type && this.update(a);
      }),
      (b.clear = function () {
        if (this.canvas) {
          var a = this.canvas.getContext("2d");
          a.setTransform(1, 0, 0, 1, 0, 0),
            a.clearRect(0, 0, this.canvas.width + 1, this.canvas.height + 1);
        }
      }),
      (b.toDataURL = function (a, b) {
        b || (b = "image/png");
        var c,
          d = this.canvas.getContext("2d"),
          e = this.canvas.width,
          f = this.canvas.height;
        if (a) {
          c = d.getImageData(0, 0, e, f);
          var g = d.globalCompositeOperation;
          (d.globalCompositeOperation = "destination-over"),
            (d.fillStyle = a),
            d.fillRect(0, 0, e, f);
        }
        var h = this.canvas.toDataURL(b);
        return (
          a &&
            (d.clearRect(0, 0, e + 1, f + 1),
            d.putImageData(c, 0, 0),
            (d.globalCompositeOperation = g)),
          h
        );
      }),
      (b.enableMouseOver = function (a) {
        if (
          (this._mouseOverIntervalID &&
            (clearInterval(this._mouseOverIntervalID),
            (this._mouseOverIntervalID = null),
            0 == a && this._testMouseOver(!0)),
          null == a)
        )
          a = 20;
        else if (0 >= a) return;
        var b = this;
        this._mouseOverIntervalID = setInterval(function () {
          b._testMouseOver();
        }, 1e3 / Math.min(50, a));
      }),
      (b.enableDOMEvents = function (a) {
        null == a && (a = !0);
        var b,
          c,
          d = this._eventListeners;
        if (!a && d) {
          for (b in d) (c = d[b]), c.t.removeEventListener(b, c.f, !1);
          this._eventListeners = null;
        } else if (a && !d && this.canvas) {
          var e = window.addEventListener ? window : document,
            f = this;
          (d = this._eventListeners = {}),
            (d.mouseup = {
              t: e,
              f: function (a) {
                f._handleMouseUp(a);
              },
            }),
            (d.mousemove = {
              t: e,
              f: function (a) {
                f._handleMouseMove(a);
              },
            }),
            (d.dblclick = {
              t: this.canvas,
              f: function (a) {
                f._handleDoubleClick(a);
              },
            }),
            (d.mousedown = {
              t: this.canvas,
              f: function (a) {
                f._handleMouseDown(a);
              },
            });
          for (b in d) (c = d[b]), c.t.addEventListener(b, c.f, !1);
        }
      }),
      (b.clone = function () {
        var b = new a(null);
        return this.cloneProps(b), b;
      }),
      (b.toString = function () {
        return "[Stage (name=" + this.name + ")]";
      }),
      (b._getElementRect = function (a) {
        var b;
        try {
          b = a.getBoundingClientRect();
        } catch (c) {
          b = {
            top: a.offsetTop,
            left: a.offsetLeft,
            width: a.offsetWidth,
            height: a.offsetHeight,
          };
        }
        var d =
            (window.pageXOffset || document.scrollLeft || 0) -
            (document.clientLeft || document.body.clientLeft || 0),
          e =
            (window.pageYOffset || document.scrollTop || 0) -
            (document.clientTop || document.body.clientTop || 0),
          f = window.getComputedStyle ? getComputedStyle(a) : a.currentStyle,
          g = parseInt(f.paddingLeft) + parseInt(f.borderLeftWidth),
          h = parseInt(f.paddingTop) + parseInt(f.borderTopWidth),
          i = parseInt(f.paddingRight) + parseInt(f.borderRightWidth),
          j = parseInt(f.paddingBottom) + parseInt(f.borderBottomWidth);
        return {
          left: b.left + d + g,
          right: b.right + d - i,
          top: b.top + e + h,
          bottom: b.bottom + e - j,
        };
      }),
      (b._getPointerData = function (a) {
        var b = this._pointerData[a];
        return (
          b ||
            ((b = this._pointerData[a] = { x: 0, y: 0 }),
            null == this._primaryPointerID && (this._primaryPointerID = a),
            (null == this._primaryPointerID || -1 == this._primaryPointerID) &&
              (this._primaryPointerID = a)),
          b
        );
      }),
      (b._handleMouseMove = function (a) {
        a || (a = window.event),
          this._handlePointerMove(-1, a, a.pageX, a.pageY);
      }),
      (b._handlePointerMove = function (a, b, c, d) {
        if (this.canvas) {
          var e = this._getPointerData(a),
            f = e.inBounds;
          if (
            (this._updatePointerPosition(a, b, c, d),
            f || e.inBounds || this.mouseMoveOutside)
          ) {
            -1 == a &&
              e.inBounds == !f &&
              this._dispatchMouseEvent(
                this,
                f ? "mouseleave" : "mouseenter",
                !1,
                a,
                e,
                b
              ),
              this._dispatchMouseEvent(this, "stagemousemove", !1, a, e, b),
              this._dispatchMouseEvent(e.target, "pressmove", !0, a, e, b);
            var g = e.event;
            g &&
              g.hasEventListener("mousemove") &&
              g.dispatchEvent(
                new createjs.MouseEvent(
                  "mousemove",
                  !1,
                  !1,
                  e.x,
                  e.y,
                  b,
                  a,
                  a == this._primaryPointerID,
                  e.rawX,
                  e.rawY
                ),
                e.target
              ),
              this.nextStage && this.nextStage._handlePointerMove(a, b, c, d);
          }
        }
      }),
      (b._updatePointerPosition = function (a, b, c, d) {
        var e = this._getElementRect(this.canvas);
        (c -= e.left), (d -= e.top);
        var f = this.canvas.width,
          g = this.canvas.height;
        (c /= (e.right - e.left) / f), (d /= (e.bottom - e.top) / g);
        var h = this._getPointerData(a);
        (h.inBounds = c >= 0 && d >= 0 && f - 1 >= c && g - 1 >= d)
          ? ((h.x = c), (h.y = d))
          : this.mouseMoveOutside &&
            ((h.x = 0 > c ? 0 : c > f - 1 ? f - 1 : c),
            (h.y = 0 > d ? 0 : d > g - 1 ? g - 1 : d)),
          (h.posEvtObj = b),
          (h.rawX = c),
          (h.rawY = d),
          a == this._primaryPointerID &&
            ((this.mouseX = h.x),
            (this.mouseY = h.y),
            (this.mouseInBounds = h.inBounds));
      }),
      (b._handleMouseUp = function (a) {
        this._handlePointerUp(-1, a, !1);
      }),
      (b._handlePointerUp = function (a, b, c) {
        var d = this._getPointerData(a);
        this._dispatchMouseEvent(this, "stagemouseup", !1, a, d, b);
        var e = d.target;
        e &&
          (this._getObjectsUnderPoint(d.x, d.y, null, !0) == e &&
            this._dispatchMouseEvent(e, "click", !0, a, d, b),
          this._dispatchMouseEvent(e, "pressup", !0, a, d, b));
        var f = d.event;
        f &&
          f.hasEventListener("mouseup") &&
          f.dispatchEvent(
            new createjs.MouseEvent(
              "mouseup",
              !1,
              !1,
              d.x,
              d.y,
              b,
              a,
              a == this._primaryPointerID,
              d.rawX,
              d.rawY
            ),
            e
          ),
          c
            ? (a == this._primaryPointerID && (this._primaryPointerID = null),
              delete this._pointerData[a])
            : (d.event = d.target = null),
          this.nextStage && this.nextStage._handlePointerUp(a, b, c);
      }),
      (b._handleMouseDown = function (a) {
        this._handlePointerDown(-1, a, a.pageX, a.pageY);
      }),
      (b._handlePointerDown = function (a, b, c, d) {
        null != d && this._updatePointerPosition(a, b, c, d);
        var e = this._getPointerData(a);
        this._dispatchMouseEvent(this, "stagemousedown", !1, a, e, b),
          (e.target = this._getObjectsUnderPoint(e.x, e.y, null, !0)),
          (e.event = this._dispatchMouseEvent(
            e.target,
            "mousedown",
            !0,
            a,
            e,
            b
          )),
          this.nextStage && this.nextStage._handlePointerDown(a, b, c, d);
      }),
      (b._testMouseOver = function (a) {
        if (
          -1 == this._primaryPointerID &&
          (a ||
            this.mouseX != this._mouseOverX ||
            this.mouseY != this._mouseOverY ||
            !this.mouseInBounds)
        ) {
          var b,
            c,
            d,
            e,
            f = this._getPointerData(-1),
            g = f.posEvtObj,
            h = -1,
            i = "";
          (a || (this.mouseInBounds && g && g.target == this.canvas)) &&
            ((b = this._getObjectsUnderPoint(
              this.mouseX,
              this.mouseY,
              null,
              !0
            )),
            (this._mouseOverX = this.mouseX),
            (this._mouseOverY = this.mouseY));
          var j = this._mouseOverTarget || [],
            k = j[j.length - 1],
            l = (this._mouseOverTarget = []);
          for (c = b; c; )
            l.unshift(c), null != c.cursor && (i = c.cursor), (c = c.parent);
          for (
            this.canvas.style.cursor = i, d = 0, e = l.length;
            e > d && l[d] == j[d];
            d++
          )
            h = d;
          for (
            k != b && this._dispatchMouseEvent(k, "mouseout", !0, -1, f, g),
              d = j.length - 1;
            d > h;
            d--
          )
            this._dispatchMouseEvent(j[d], "rollout", !1, -1, f, g);
          for (d = l.length - 1; d > h; d--)
            this._dispatchMouseEvent(l[d], "rollover", !1, -1, f, g);
          k != b && this._dispatchMouseEvent(b, "mouseover", !0, -1, f, g);
        }
      }),
      (b._handleDoubleClick = function (a) {
        var b = this._getPointerData(-1),
          c = this._getObjectsUnderPoint(b.x, b.y, null, !0);
        this._dispatchMouseEvent(c, "dblclick", !0, -1, b, a),
          this.nextStage && this.nextStage._handleDoubleClick(a);
      }),
      (b._dispatchMouseEvent = function (a, b, c, d, e, f) {
        if (a && (c || a.hasEventListener(b))) {
          var g = new createjs.MouseEvent(
            b,
            c,
            !1,
            e.x,
            e.y,
            f,
            d,
            d == this._primaryPointerID,
            e.rawX,
            e.rawY
          );
          return a.dispatchEvent(g), g;
        }
      }),
      (createjs.Stage = a);
  })(),
  (this.createjs = this.createjs || {}),
  (function () {
    var a = function (a) {
        this.initialize(a);
      },
      b = (a.prototype = new createjs.DisplayObject());
    (b.image = null),
      (b.snapToPixel = !0),
      (b.sourceRect = null),
      (b.DisplayObject_initialize = b.initialize),
      (b.initialize = function (a) {
        this.DisplayObject_initialize(),
          "string" == typeof a
            ? ((this.image = document.createElement("img")),
              (this.image.src = a))
            : (this.image = a);
      }),
      (b.isVisible = function () {
        var a =
          this.cacheCanvas ||
          (this.image &&
            (this.image.complete ||
              this.image.getContext ||
              this.image.readyState >= 2));
        return !!(
          this.visible &&
          this.alpha > 0 &&
          0 != this.scaleX &&
          0 != this.scaleY &&
          a
        );
      }),
      (b.DisplayObject_draw = b.draw),
      (b.draw = function (a, b) {
        if (this.DisplayObject_draw(a, b)) return !0;
        var c = this.sourceRect;
        return (
          c
            ? a.drawImage(
                this.image,
                c.x,
                c.y,
                c.width,
                c.height,
                0,
                0,
                c.width,
                c.height
              )
            : a.drawImage(this.image, 0, 0),
          !0
        );
      }),
      (b.DisplayObject_getBounds = b.getBounds),
      (b.getBounds = function () {
        var a = this.DisplayObject_getBounds();
        if (a) return a;
        var b = this.sourceRect || this.image,
          c =
            this.image &&
            (this.image.complete ||
              this.image.getContext ||
              this.image.readyState >= 2);
        return c ? this._rectangle.initialize(0, 0, b.width, b.height) : null;
      }),
      (b.clone = function () {
        var b = new a(this.image);
        return (
          this.sourceRect && (b.sourceRect = this.sourceRect.clone()),
          this.cloneProps(b),
          b
        );
      }),
      (b.toString = function () {
        return "[Bitmap (name=" + this.name + ")]";
      }),
      (createjs.Bitmap = a);
  })(),
  (this.createjs = this.createjs || {}),
  (function () {
    "use strict";
    var a = function (a, b) {
        this.initialize(a, b);
      },
      b = (a.prototype = new createjs.DisplayObject());
    (b.currentFrame = 0),
      (b.currentAnimation = null),
      (b.paused = !0),
      (b.spriteSheet = null),
      (b.snapToPixel = !0),
      (b.offset = 0),
      (b.currentAnimationFrame = 0),
      (b.framerate = 0),
      (b._advanceCount = 0),
      (b._animation = null),
      (b._currentFrame = null),
      (b.DisplayObject_initialize = b.initialize),
      (b.initialize = function (a, b) {
        this.DisplayObject_initialize(),
          (this.spriteSheet = a),
          b && this.gotoAndPlay(b);
      }),
      (b.isVisible = function () {
        var a = this.cacheCanvas || this.spriteSheet.complete;
        return !!(
          this.visible &&
          this.alpha > 0 &&
          0 != this.scaleX &&
          0 != this.scaleY &&
          a
        );
      }),
      (b.DisplayObject_draw = b.draw),
      (b.draw = function (a, b) {
        if (this.DisplayObject_draw(a, b)) return !0;
        this._normalizeFrame();
        var c = this.spriteSheet.getFrame(0 | this._currentFrame);
        if (!c) return !1;
        var d = c.rect;
        return (
          a.drawImage(
            c.image,
            d.x,
            d.y,
            d.width,
            d.height,
            -c.regX,
            -c.regY,
            d.width,
            d.height
          ),
          !0
        );
      }),
      (b.play = function () {
        this.paused = !1;
      }),
      (b.stop = function () {
        this.paused = !0;
      }),
      (b.gotoAndPlay = function (a) {
        (this.paused = !1), this._goto(a);
      }),
      (b.gotoAndStop = function (a) {
        (this.paused = !0), this._goto(a);
      }),
      (b.advance = function (a) {
        var b = (this._animation && this._animation.speed) || 1,
          c = this.framerate || this.spriteSheet.framerate,
          d = c && null != a ? a / (1e3 / c) : 1;
        this._animation
          ? (this.currentAnimationFrame += d * b)
          : (this._currentFrame += d * b),
          this._normalizeFrame();
      }),
      (b.DisplayObject_getBounds = b.getBounds),
      (b.getBounds = function () {
        return (
          this.DisplayObject_getBounds() ||
          this.spriteSheet.getFrameBounds(this.currentFrame, this._rectangle)
        );
      }),
      (b.clone = function () {
        var b = new a(this.spriteSheet);
        return this.cloneProps(b), b;
      }),
      (b.toString = function () {
        return "[Sprite (name=" + this.name + ")]";
      }),
      (b.DisplayObject__tick = b._tick),
      (b._tick = function (a) {
        this.paused || this.advance(a && a[0] && a[0].delta),
          this.DisplayObject__tick(a);
      }),
      (b._normalizeFrame = function () {
        var a,
          b = this._animation,
          c = this.paused,
          d = this._currentFrame,
          e = this.currentAnimationFrame;
        if (b)
          if (((a = b.frames.length), (0 | e) >= a)) {
            var f = b.next;
            if (this._dispatchAnimationEnd(b, d, c, f, a - 1));
            else {
              if (f) return this._goto(f, e - a);
              (this.paused = !0),
                (e = this.currentAnimationFrame = b.frames.length - 1),
                (this._currentFrame = b.frames[e]);
            }
          } else this._currentFrame = b.frames[0 | e];
        else if (
          ((a = this.spriteSheet.getNumFrames()),
          d >= a &&
            !this._dispatchAnimationEnd(b, d, c, a - 1) &&
            (this._currentFrame -= a) >= a)
        )
          return this._normalizeFrame();
        this.currentFrame = 0 | this._currentFrame;
      }),
      (b._dispatchAnimationEnd = function (a, b, c, d, e) {
        var f = a ? a.name : null;
        if (this.hasEventListener("animationend")) {
          var g = new createjs.Event("animationend");
          (g.name = f), (g.next = d), this.dispatchEvent(g);
        }
        var h = this._animation != a || this._currentFrame != b;
        return (
          h ||
            c ||
            !this.paused ||
            ((this.currentAnimationFrame = e), (h = !0)),
          h
        );
      }),
      (b.DisplayObject_cloneProps = b.cloneProps),
      (b.cloneProps = function (a) {
        this.DisplayObject_cloneProps(a),
          (a.currentFrame = this.currentFrame),
          (a._currentFrame = this._currentFrame),
          (a.currentAnimation = this.currentAnimation),
          (a.paused = this.paused),
          (a._animation = this._animation),
          (a.currentAnimationFrame = this.currentAnimationFrame),
          (a.framerate = this.framerate);
      }),
      (b._goto = function (a, b) {
        if (isNaN(a)) {
          var c = this.spriteSheet.getAnimation(a);
          c &&
            ((this.currentAnimationFrame = b || 0),
            (this._animation = c),
            (this.currentAnimation = a),
            this._normalizeFrame());
        } else
          (this.currentAnimationFrame = 0),
            (this.currentAnimation = this._animation = null),
            (this._currentFrame = a),
            this._normalizeFrame();
      }),
      (createjs.Sprite = a);
  })(),
  (this.createjs = this.createjs || {}),
  (function () {
    "use strict";
    var a =
      "BitmapAnimation is deprecated in favour of Sprite. See VERSIONS file for info on changes.";
    if (!createjs.Sprite) throw a;
    (createjs.BitmapAnimation = function (b) {
      console.log(a), this.initialize(b);
    }).prototype = new createjs.Sprite();
  })(),
  (this.createjs = this.createjs || {}),
  (function () {
    "use strict";
    var a = function (a) {
        this.initialize(a);
      },
      b = (a.prototype = new createjs.DisplayObject());
    (b.graphics = null),
      (b.DisplayObject_initialize = b.initialize),
      (b.initialize = function (a) {
        this.DisplayObject_initialize(),
          (this.graphics = a ? a : new createjs.Graphics());
      }),
      (b.isVisible = function () {
        var a = this.cacheCanvas || (this.graphics && !this.graphics.isEmpty());
        return !!(
          this.visible &&
          this.alpha > 0 &&
          0 != this.scaleX &&
          0 != this.scaleY &&
          a
        );
      }),
      (b.DisplayObject_draw = b.draw),
      (b.draw = function (a, b) {
        return this.DisplayObject_draw(a, b) ? !0 : (this.graphics.draw(a), !0);
      }),
      (b.clone = function (b) {
        var c = new a(
          b && this.graphics ? this.graphics.clone() : this.graphics
        );
        return this.cloneProps(c), c;
      }),
      (b.toString = function () {
        return "[Shape (name=" + this.name + ")]";
      }),
      (createjs.Shape = a);
  })(),
  (this.createjs = this.createjs || {}),
  (function () {
    "use strict";
    var a = function (a, b, c) {
        this.initialize(a, b, c);
      },
      b = (a.prototype = new createjs.DisplayObject()),
      c = createjs.createCanvas
        ? createjs.createCanvas()
        : document.createElement("canvas");
    c.getContext &&
      ((a._workingContext = c.getContext("2d")), (c.width = c.height = 1)),
      (a.H_OFFSETS = { start: 0, left: 0, center: -0.5, end: -1, right: -1 }),
      (a.V_OFFSETS = {
        top: 0,
        hanging: -0.01,
        middle: -0.4,
        alphabetic: -0.8,
        ideographic: -0.85,
        bottom: -1,
      }),
      (b.text = ""),
      (b.font = null),
      (b.color = null),
      (b.textAlign = "left"),
      (b.textBaseline = "top"),
      (b.maxWidth = null),
      (b.outline = 0),
      (b.lineHeight = 0),
      (b.lineWidth = null),
      (b.DisplayObject_initialize = b.initialize),
      (b.initialize = function (a, b, c) {
        this.DisplayObject_initialize(),
          (this.text = a),
          (this.font = b),
          (this.color = c);
      }),
      (b.isVisible = function () {
        var a = this.cacheCanvas || (null != this.text && "" !== this.text);
        return !!(
          this.visible &&
          this.alpha > 0 &&
          0 != this.scaleX &&
          0 != this.scaleY &&
          a
        );
      }),
      (b.DisplayObject_draw = b.draw),
      (b.draw = function (a, b) {
        if (this.DisplayObject_draw(a, b)) return !0;
        var c = this.color || "#000";
        return (
          this.outline
            ? ((a.strokeStyle = c), (a.lineWidth = 1 * this.outline))
            : (a.fillStyle = c),
          this._drawText(this._prepContext(a)),
          !0
        );
      }),
      (b.getMeasuredWidth = function () {
        return this._prepContext(a._workingContext).measureText(this.text)
          .width;
      }),
      (b.getMeasuredLineHeight = function () {
        return (
          1.2 * this._prepContext(a._workingContext).measureText("M").width
        );
      }),
      (b.getMeasuredHeight = function () {
        return this._drawText(null, {}).height;
      }),
      (b.DisplayObject_getBounds = b.getBounds),
      (b.getBounds = function () {
        var b = this.DisplayObject_getBounds();
        if (b) return b;
        if (null == this.text || "" == this.text) return null;
        var c = this._drawText(null, {}),
          d =
            this.maxWidth && this.maxWidth < c.width ? this.maxWidth : c.width,
          e = d * a.H_OFFSETS[this.textAlign || "left"],
          f = this.lineHeight || this.getMeasuredLineHeight(),
          g = f * a.V_OFFSETS[this.textBaseline || "top"];
        return this._rectangle.initialize(e, g, d, c.height);
      }),
      (b.clone = function () {
        var b = new a(this.text, this.font, this.color);
        return this.cloneProps(b), b;
      }),
      (b.toString = function () {
        return (
          "[Text (text=" +
          (this.text.length > 20
            ? this.text.substr(0, 17) + "..."
            : this.text) +
          ")]"
        );
      }),
      (b.DisplayObject_cloneProps = b.cloneProps),
      (b.cloneProps = function (a) {
        this.DisplayObject_cloneProps(a),
          (a.textAlign = this.textAlign),
          (a.textBaseline = this.textBaseline),
          (a.maxWidth = this.maxWidth),
          (a.outline = this.outline),
          (a.lineHeight = this.lineHeight),
          (a.lineWidth = this.lineWidth);
      }),
      (b._prepContext = function (a) {
        return (
          (a.font = this.font),
          (a.textAlign = this.textAlign || "left"),
          (a.textBaseline = this.textBaseline || "top"),
          a
        );
      }),
      (b._drawText = function (b, c) {
        var d = !!b;
        d || (b = this._prepContext(a._workingContext));
        for (
          var e = this.lineHeight || this.getMeasuredLineHeight(),
            f = 0,
            g = 0,
            h = String(this.text).split(/(?:\r\n|\r|\n)/),
            i = 0,
            j = h.length;
          j > i;
          i++
        ) {
          var k = h[i],
            l = null;
          if (
            null != this.lineWidth &&
            (l = b.measureText(k).width) > this.lineWidth
          ) {
            var m = k.split(/(\s)/);
            (k = m[0]), (l = b.measureText(k).width);
            for (var n = 1, o = m.length; o > n; n += 2) {
              var p = b.measureText(m[n] + m[n + 1]).width;
              l + p > this.lineWidth
                ? (d && this._drawTextLine(b, k, g * e),
                  l > f && (f = l),
                  (k = m[n + 1]),
                  (l = b.measureText(k).width),
                  g++)
                : ((k += m[n] + m[n + 1]), (l += p));
            }
          }
          d && this._drawTextLine(b, k, g * e),
            c && null == l && (l = b.measureText(k).width),
            l > f && (f = l),
            g++;
        }
        return c && ((c.count = g), (c.width = f), (c.height = g * e)), c;
      }),
      (b._drawTextLine = function (a, b, c) {
        this.outline
          ? a.strokeText(b, 0, c, this.maxWidth || 65535)
          : a.fillText(b, 0, c, this.maxWidth || 65535);
      }),
      (createjs.Text = a);
  })(),
  (this.createjs = this.createjs || {}),
  (function () {
    "use strict";
    function a(a, b) {
      this.initialize(a, b);
    }
    var b = (a.prototype = new createjs.DisplayObject());
    (b.text = ""),
      (b.spriteSheet = null),
      (b.lineHeight = 0),
      (b.letterSpacing = 0),
      (b.spaceWidth = 0),
      (b.DisplayObject_initialize = b.initialize),
      (b.initialize = function (a, b) {
        this.DisplayObject_initialize(),
          (this.text = a),
          (this.spriteSheet = b);
      }),
      (b.DisplayObject_draw = b.draw),
      (b.draw = function (a, b) {
        return this.DisplayObject_draw(a, b) ? !0 : (this._drawText(a), void 0);
      }),
      (b.isVisible = function () {
        var a =
          this.cacheCanvas ||
          (this.spriteSheet && this.spriteSheet.complete && this.text);
        return !!(
          this.visible &&
          this.alpha > 0 &&
          0 != this.scaleX &&
          0 != this.scaleY &&
          a
        );
      }),
      (b.getBounds = function () {
        var a = this._rectangle;
        return this._drawText(null, a), a.width ? a : null;
      }),
      (b._getFrame = function (a, b) {
        var c,
          d = b.getAnimation(a);
        return (
          d ||
            (a != (c = a.toUpperCase()) ||
              a != (c = a.toLowerCase()) ||
              (c = null),
            c && (d = b.getAnimation(c))),
          d && b.getFrame(d.frames[0])
        );
      }),
      (b._getLineHeight = function (a) {
        var b =
          this._getFrame("1", a) ||
          this._getFrame("T", a) ||
          this._getFrame("L", a) ||
          a.getFrame(0);
        return b ? b.rect.height : 1;
      }),
      (b._getSpaceWidth = function (a) {
        var b =
          this._getFrame("1", a) ||
          this._getFrame("l", a) ||
          this._getFrame("e", a) ||
          this._getFrame("a", a) ||
          a.getFrame(0);
        return b ? b.rect.width : 1;
      }),
      (b._drawText = function (a, b) {
        var c,
          d,
          e,
          f = 0,
          g = 0,
          h = this.spaceWidth,
          i = this.lineHeight,
          j = this.spriteSheet,
          k = !!this._getFrame(" ", j);
        k || 0 != h || (h = this._getSpaceWidth(j)),
          0 == i && (i = this._getLineHeight(j));
        for (var l = 0, m = 0, n = this.text.length; n > m; m++) {
          var o = this.text.charAt(m);
          if (k || " " != o)
            if ("\n" != o && "\r" != o) {
              var p = this._getFrame(o, j);
              if (p) {
                var q = p.rect;
                (e = p.regX),
                  (c = q.width),
                  a &&
                    a.drawImage(
                      p.image,
                      q.x,
                      q.y,
                      c,
                      (d = q.height),
                      f - e,
                      g - p.regY,
                      c,
                      d
                    ),
                  (f += c + this.letterSpacing);
              }
            } else
              "\r" == o && "\n" == this.text.charAt(m + 1) && m++,
                f - e > l && (l = f - e),
                (f = 0),
                (g += i);
          else f += h;
        }
        f - e > l && (l = f - e),
          b && ((b.width = l - this.letterSpacing), (b.height = g + i));
      }),
      (createjs.BitmapText = a);
  })(),
  (this.createjs = this.createjs || {}),
  (function () {
    "use strict";
    var a = function () {
        throw "SpriteSheetUtils cannot be instantiated";
      },
      b = createjs.createCanvas
        ? createjs.createCanvas()
        : document.createElement("canvas");
    b.getContext &&
      ((a._workingCanvas = b),
      (a._workingContext = b.getContext("2d")),
      (b.width = b.height = 1)),
      (a.addFlippedFrames = function (b, c, d, e) {
        if (c || d || e) {
          var f = 0;
          c && a._flip(b, ++f, !0, !1),
            d && a._flip(b, ++f, !1, !0),
            e && a._flip(b, ++f, !0, !0);
        }
      }),
      (a.extractFrame = function (b, c) {
        isNaN(c) && (c = b.getAnimation(c).frames[0]);
        var d = b.getFrame(c);
        if (!d) return null;
        var e = d.rect,
          f = a._workingCanvas;
        (f.width = e.width),
          (f.height = e.height),
          a._workingContext.drawImage(
            d.image,
            e.x,
            e.y,
            e.width,
            e.height,
            0,
            0,
            e.width,
            e.height
          );
        var g = document.createElement("img");
        return (g.src = f.toDataURL("image/png")), g;
      }),
      (a.mergeAlpha = function (a, b, c) {
        c ||
          (c = createjs.createCanvas
            ? createjs.createCanvas()
            : document.createElement("canvas")),
          (c.width = Math.max(b.width, a.width)),
          (c.height = Math.max(b.height, a.height));
        var d = c.getContext("2d");
        return (
          d.save(),
          d.drawImage(a, 0, 0),
          (d.globalCompositeOperation = "destination-in"),
          d.drawImage(b, 0, 0),
          d.restore(),
          c
        );
      }),
      (a._flip = function (b, c, d, e) {
        for (
          var f = b._images,
            g = a._workingCanvas,
            h = a._workingContext,
            i = f.length / c,
            j = 0;
          i > j;
          j++
        ) {
          var k = f[j];
          (k.__tmp = j),
            h.setTransform(1, 0, 0, 1, 0, 0),
            h.clearRect(0, 0, g.width + 1, g.height + 1),
            (g.width = k.width),
            (g.height = k.height),
            h.setTransform(
              d ? -1 : 1,
              0,
              0,
              e ? -1 : 1,
              d ? k.width : 0,
              e ? k.height : 0
            ),
            h.drawImage(k, 0, 0);
          var l = document.createElement("img");
          (l.src = g.toDataURL("image/png")),
            (l.width = k.width),
            (l.height = k.height),
            f.push(l);
        }
        var m = b._frames,
          n = m.length / c;
        for (j = 0; n > j; j++) {
          k = m[j];
          var o = k.rect.clone();
          l = f[k.image.__tmp + i * c];
          var p = { image: l, rect: o, regX: k.regX, regY: k.regY };
          d && ((o.x = l.width - o.x - o.width), (p.regX = o.width - k.regX)),
            e &&
              ((o.y = l.height - o.y - o.height), (p.regY = o.height - k.regY)),
            m.push(p);
        }
        var q = "_" + (d ? "h" : "") + (e ? "v" : ""),
          r = b._animations,
          s = b._data,
          t = r.length / c;
        for (j = 0; t > j; j++) {
          var u = r[j];
          k = s[u];
          var v = { name: u + q, speed: k.speed, next: k.next, frames: [] };
          k.next && (v.next += q), (m = k.frames);
          for (var w = 0, x = m.length; x > w; w++) v.frames.push(m[w] + n * c);
          (s[v.name] = v), r.push(v.name);
        }
      }),
      (createjs.SpriteSheetUtils = a);
  })(),
  (this.createjs = this.createjs || {}),
  (function () {
    "use strict";
    var a = function () {
        this.initialize();
      },
      b = (a.prototype = new createjs.EventDispatcher());
    (a.ERR_DIMENSIONS = "frame dimensions exceed max spritesheet dimensions"),
      (a.ERR_RUNNING = "a build is already running"),
      (b.maxWidth = 2048),
      (b.maxHeight = 2048),
      (b.spriteSheet = null),
      (b.scale = 1),
      (b.padding = 1),
      (b.timeSlice = 0.3),
      (b.progress = -1),
      (b._frames = null),
      (b._animations = null),
      (b._data = null),
      (b._nextFrameIndex = 0),
      (b._index = 0),
      (b._timerID = null),
      (b._scale = 1),
      (b.initialize = function () {
        (this._frames = []), (this._animations = {});
      }),
      (b.addFrame = function (b, c, d, e, f, g) {
        if (this._data) throw a.ERR_RUNNING;
        var h = c || b.bounds || b.nominalBounds;
        return (
          !h && b.getBounds && (h = b.getBounds()),
          h
            ? ((d = d || 1),
              this._frames.push({
                source: b,
                sourceRect: h,
                scale: d,
                funct: e,
                params: f,
                scope: g,
                index: this._frames.length,
                height: h.height * d,
              }) - 1)
            : null
        );
      }),
      (b.addAnimation = function (b, c, d, e) {
        if (this._data) throw a.ERR_RUNNING;
        this._animations[b] = { frames: c, next: d, frequency: e };
      }),
      (b.addMovieClip = function (b, c, d) {
        if (this._data) throw a.ERR_RUNNING;
        var e = b.frameBounds,
          f = c || b.bounds || b.nominalBounds;
        if ((!f && b.getBounds && (f = b.getBounds()), !f && !e)) return null;
        for (
          var g = this._frames.length, h = b.timeline.duration, i = 0;
          h > i;
          i++
        ) {
          var j = e && e[i] ? e[i] : f;
          this.addFrame(
            b,
            j,
            d,
            function (a) {
              var b = this.actionsEnabled;
              (this.actionsEnabled = !1),
                this.gotoAndStop(a),
                (this.actionsEnabled = b);
            },
            [i],
            b
          );
        }
        var k = b.timeline._labels,
          l = [];
        for (var m in k) l.push({ index: k[m], label: m });
        if (l.length) {
          l.sort(function (a, b) {
            return a.index - b.index;
          });
          for (var i = 0, n = l.length; n > i; i++) {
            for (
              var o = l[i].label,
                p = g + l[i].index,
                q = g + (i == n - 1 ? h : l[i + 1].index),
                r = [],
                s = p;
              q > s;
              s++
            )
              r.push(s);
            this.addAnimation(o, r, !0);
          }
        }
      }),
      (b.build = function () {
        if (this._data) throw a.ERR_RUNNING;
        for (this._startBuild(); this._drawNext(); );
        return this._endBuild(), this.spriteSheet;
      }),
      (b.buildAsync = function (b) {
        if (this._data) throw a.ERR_RUNNING;
        (this.timeSlice = b), this._startBuild();
        var c = this;
        this._timerID = setTimeout(function () {
          c._run();
        }, 50 - 50 * Math.max(0.01, Math.min(0.99, this.timeSlice || 0.3)));
      }),
      (b.stopAsync = function () {
        clearTimeout(this._timerID), (this._data = null);
      }),
      (b.clone = function () {
        throw "SpriteSheetBuilder cannot be cloned.";
      }),
      (b.toString = function () {
        return "[SpriteSheetBuilder]";
      }),
      (b._startBuild = function () {
        var b = this.padding || 0;
        (this.progress = 0),
          (this.spriteSheet = null),
          (this._index = 0),
          (this._scale = this.scale);
        var c = [];
        this._data = { images: [], frames: c, animations: this._animations };
        var d = this._frames.slice();
        if (
          (d.sort(function (a, b) {
            return a.height <= b.height ? -1 : 1;
          }),
          d[d.length - 1].height + 2 * b > this.maxHeight)
        )
          throw a.ERR_DIMENSIONS;
        for (var e = 0, f = 0, g = 0; d.length; ) {
          var h = this._fillRow(d, e, g, c, b);
          if ((h.w > f && (f = h.w), (e += h.h), !h.h || !d.length)) {
            var i = createjs.createCanvas
              ? createjs.createCanvas()
              : document.createElement("canvas");
            (i.width = this._getSize(f, this.maxWidth)),
              (i.height = this._getSize(e, this.maxHeight)),
              (this._data.images[g] = i),
              h.h || ((f = e = 0), g++);
          }
        }
      }),
      (b._getSize = function (a, b) {
        for (var c = 4; Math.pow(2, ++c) < a; );
        return Math.min(b, Math.pow(2, c));
      }),
      (b._fillRow = function (b, c, d, e, f) {
        var g = this.maxWidth,
          h = this.maxHeight;
        c += f;
        for (var i = h - c, j = f, k = 0, l = b.length - 1; l >= 0; l--) {
          var m = b[l],
            n = this._scale * m.scale,
            o = m.sourceRect,
            p = m.source,
            q = Math.floor(n * o.x - f),
            r = Math.floor(n * o.y - f),
            s = Math.ceil(n * o.height + 2 * f),
            t = Math.ceil(n * o.width + 2 * f);
          if (t > g) throw a.ERR_DIMENSIONS;
          s > i ||
            j + t > g ||
            ((m.img = d),
            (m.rect = new createjs.Rectangle(j, c, t, s)),
            (k = k || s),
            b.splice(l, 1),
            (e[m.index] = [
              j,
              c,
              t,
              s,
              d,
              Math.round(-q + n * p.regX - f),
              Math.round(-r + n * p.regY - f),
            ]),
            (j += t));
        }
        return { w: j, h: k };
      }),
      (b._endBuild = function () {
        (this.spriteSheet = new createjs.SpriteSheet(this._data)),
          (this._data = null),
          (this.progress = 1),
          this.dispatchEvent("complete");
      }),
      (b._run = function () {
        for (
          var a = 50 * Math.max(0.01, Math.min(0.99, this.timeSlice || 0.3)),
            b = new Date().getTime() + a,
            c = !1;
          b > new Date().getTime();

        )
          if (!this._drawNext()) {
            c = !0;
            break;
          }
        if (c) this._endBuild();
        else {
          var d = this;
          this._timerID = setTimeout(function () {
            d._run();
          }, 50 - a);
        }
        var e = (this.progress = this._index / this._frames.length);
        if (this.hasEventListener("progress")) {
          var f = new createjs.Event("progress");
          (f.progress = e), this.dispatchEvent(f);
        }
      }),
      (b._drawNext = function () {
        var a = this._frames[this._index],
          b = a.scale * this._scale,
          c = a.rect,
          d = a.sourceRect,
          e = this._data.images[a.img],
          f = e.getContext("2d");
        return (
          a.funct && a.funct.apply(a.scope, a.params),
          f.save(),
          f.beginPath(),
          f.rect(c.x, c.y, c.width, c.height),
          f.clip(),
          f.translate(Math.ceil(c.x - d.x * b), Math.ceil(c.y - d.y * b)),
          f.scale(b, b),
          a.source.draw(f),
          f.restore(),
          ++this._index < this._frames.length
        );
      }),
      (createjs.SpriteSheetBuilder = a);
  })(),
  (this.createjs = this.createjs || {}),
  (function () {
    "use strict";
    var a = function (a) {
        this.initialize(a);
      },
      b = (a.prototype = new createjs.DisplayObject());
    (b.htmlElement = null),
      (b._oldMtx = null),
      (b._visible = !1),
      (b.DisplayObject_initialize = b.initialize),
      (b.initialize = function (a) {
        "string" == typeof a && (a = document.getElementById(a)),
          this.DisplayObject_initialize(),
          (this.mouseEnabled = !1),
          (this.htmlElement = a);
        var b = a.style;
        (b.position = "absolute"),
          (b.transformOrigin = b.WebkitTransformOrigin = b.msTransformOrigin = b.MozTransformOrigin = b.OTransformOrigin =
            "0% 0%");
      }),
      (b.isVisible = function () {
        return null != this.htmlElement;
      }),
      (b.draw = function () {
        return this.visible && (this._visible = !0), !0;
      }),
      (b.cache = function () {}),
      (b.uncache = function () {}),
      (b.updateCache = function () {}),
      (b.hitTest = function () {}),
      (b.localToGlobal = function () {}),
      (b.globalToLocal = function () {}),
      (b.localToLocal = function () {}),
      (b.clone = function () {
        throw "DOMElement cannot be cloned.";
      }),
      (b.toString = function () {
        return "[DOMElement (name=" + this.name + ")]";
      }),
      (b.DisplayObject__tick = b._tick),
      (b._tick = function (a) {
        var b = this.getStage();
        (this._visible = !1),
          b && b.on("drawend", this._handleDrawEnd, this, !0),
          this.DisplayObject__tick(a);
      }),
      (b._handleDrawEnd = function () {
        var a = this.htmlElement;
        if (a) {
          var b = a.style,
            c = this._visible ? "visible" : "hidden";
          if ((c != b.visibility && (b.visibility = c), this._visible)) {
            var d = this.getConcatenatedMatrix(this._matrix),
              e = this._oldMtx,
              f = 1e4;
            if (
              ((e && e.alpha == d.alpha) ||
                ((b.opacity = "" + (0 | (d.alpha * f)) / f),
                e && (e.alpha = d.alpha)),
              !e ||
                e.tx != d.tx ||
                e.ty != d.ty ||
                e.a != d.a ||
                e.b != d.b ||
                e.c != d.c ||
                e.d != d.d)
            ) {
              var g =
                "matrix(" +
                (0 | (d.a * f)) / f +
                "," +
                (0 | (d.b * f)) / f +
                "," +
                (0 | (d.c * f)) / f +
                "," +
                (0 | (d.d * f)) / f +
                "," +
                (0 | (d.tx + 0.5));
              (b.transform = b.WebkitTransform = b.OTransform = b.msTransform =
                g + "," + (0 | (d.ty + 0.5)) + ")"),
                (b.MozTransform = g + "px," + (0 | (d.ty + 0.5)) + "px)"),
                (this._oldMtx = e ? e.copy(d) : d.clone());
            }
          }
        }
      }),
      (createjs.DOMElement = a);
  })(),
  (this.createjs = this.createjs || {}),
  (function () {
    "use strict";
    var a = function () {
        this.initialize();
      },
      b = a.prototype;
    (b.initialize = function () {}),
      (b.getBounds = function () {
        return null;
      }),
      (b.applyFilter = function () {}),
      (b.toString = function () {
        return "[Filter]";
      }),
      (b.clone = function () {
        return new a();
      }),
      (createjs.Filter = a);
  })(),
  (this.createjs = this.createjs || {}),
  (function () {
    "use strict";
    var a = function (a, b, c) {
        this.initialize(a, b, c);
      },
      b = (a.prototype = new createjs.Filter());
    (b.initialize = function (a, b, c) {
      (isNaN(a) || 0 > a) && (a = 0),
        (this.blurX = 0 | a),
        (isNaN(b) || 0 > b) && (b = 0),
        (this.blurY = 0 | b),
        (isNaN(c) || 1 > c) && (c = 1),
        (this.quality = 0 | c);
    }),
      (b.blurX = 0),
      (b.blurY = 0),
      (b.quality = 1),
      (b.mul_table = [
        1,
        171,
        205,
        293,
        57,
        373,
        79,
        137,
        241,
        27,
        391,
        357,
        41,
        19,
        283,
        265,
        497,
        469,
        443,
        421,
        25,
        191,
        365,
        349,
        335,
        161,
        155,
        149,
        9,
        278,
        269,
        261,
        505,
        245,
        475,
        231,
        449,
        437,
        213,
        415,
        405,
        395,
        193,
        377,
        369,
        361,
        353,
        345,
        169,
        331,
        325,
        319,
        313,
        307,
        301,
        37,
        145,
        285,
        281,
        69,
        271,
        267,
        263,
        259,
        509,
        501,
        493,
        243,
        479,
        118,
        465,
        459,
        113,
        446,
        55,
        435,
        429,
        423,
        209,
        413,
        51,
        403,
        199,
        393,
        97,
        3,
        379,
        375,
        371,
        367,
        363,
        359,
        355,
        351,
        347,
        43,
        85,
        337,
        333,
        165,
        327,
        323,
        5,
        317,
        157,
        311,
        77,
        305,
        303,
        75,
        297,
        294,
        73,
        289,
        287,
        71,
        141,
        279,
        277,
        275,
        68,
        135,
        67,
        133,
        33,
        262,
        260,
        129,
        511,
        507,
        503,
        499,
        495,
        491,
        61,
        121,
        481,
        477,
        237,
        235,
        467,
        232,
        115,
        457,
        227,
        451,
        7,
        445,
        221,
        439,
        218,
        433,
        215,
        427,
        425,
        211,
        419,
        417,
        207,
        411,
        409,
        203,
        202,
        401,
        399,
        396,
        197,
        49,
        389,
        387,
        385,
        383,
        95,
        189,
        47,
        187,
        93,
        185,
        23,
        183,
        91,
        181,
        45,
        179,
        89,
        177,
        11,
        175,
        87,
        173,
        345,
        343,
        341,
        339,
        337,
        21,
        167,
        83,
        331,
        329,
        327,
        163,
        81,
        323,
        321,
        319,
        159,
        79,
        315,
        313,
        39,
        155,
        309,
        307,
        153,
        305,
        303,
        151,
        75,
        299,
        149,
        37,
        295,
        147,
        73,
        291,
        145,
        289,
        287,
        143,
        285,
        71,
        141,
        281,
        35,
        279,
        139,
        69,
        275,
        137,
        273,
        17,
        271,
        135,
        269,
        267,
        133,
        265,
        33,
        263,
        131,
        261,
        130,
        259,
        129,
        257,
        1,
      ]),
      (b.shg_table = [
        0,
        9,
        10,
        11,
        9,
        12,
        10,
        11,
        12,
        9,
        13,
        13,
        10,
        9,
        13,
        13,
        14,
        14,
        14,
        14,
        10,
        13,
        14,
        14,
        14,
        13,
        13,
        13,
        9,
        14,
        14,
        14,
        15,
        14,
        15,
        14,
        15,
        15,
        14,
        15,
        15,
        15,
        14,
        15,
        15,
        15,
        15,
        15,
        14,
        15,
        15,
        15,
        15,
        15,
        15,
        12,
        14,
        15,
        15,
        13,
        15,
        15,
        15,
        15,
        16,
        16,
        16,
        15,
        16,
        14,
        16,
        16,
        14,
        16,
        13,
        16,
        16,
        16,
        15,
        16,
        13,
        16,
        15,
        16,
        14,
        9,
        16,
        16,
        16,
        16,
        16,
        16,
        16,
        16,
        16,
        13,
        14,
        16,
        16,
        15,
        16,
        16,
        10,
        16,
        15,
        16,
        14,
        16,
        16,
        14,
        16,
        16,
        14,
        16,
        16,
        14,
        15,
        16,
        16,
        16,
        14,
        15,
        14,
        15,
        13,
        16,
        16,
        15,
        17,
        17,
        17,
        17,
        17,
        17,
        14,
        15,
        17,
        17,
        16,
        16,
        17,
        16,
        15,
        17,
        16,
        17,
        11,
        17,
        16,
        17,
        16,
        17,
        16,
        17,
        17,
        16,
        17,
        17,
        16,
        17,
        17,
        16,
        16,
        17,
        17,
        17,
        16,
        14,
        17,
        17,
        17,
        17,
        15,
        16,
        14,
        16,
        15,
        16,
        13,
        16,
        15,
        16,
        14,
        16,
        15,
        16,
        12,
        16,
        15,
        16,
        17,
        17,
        17,
        17,
        17,
        13,
        16,
        15,
        17,
        17,
        17,
        16,
        15,
        17,
        17,
        17,
        16,
        15,
        17,
        17,
        14,
        16,
        17,
        17,
        16,
        17,
        17,
        16,
        15,
        17,
        16,
        14,
        17,
        16,
        15,
        17,
        16,
        17,
        17,
        16,
        17,
        15,
        16,
        17,
        14,
        17,
        16,
        15,
        17,
        16,
        17,
        13,
        17,
        16,
        17,
        17,
        16,
        17,
        14,
        17,
        16,
        17,
        16,
        17,
        16,
        17,
        9,
      ]),
      (b.getBounds = function () {
        var a = 0.5 * Math.pow(this.quality, 0.6);
        return new createjs.Rectangle(
          -this.blurX * a,
          -this.blurY * a,
          2 * this.blurX * a,
          2 * this.blurY * a
        );
      }),
      (b.applyFilter = function (a, b, c, d, e, f, g, h) {
        (f = f || a), null == g && (g = b), null == h && (h = c);
        try {
          var i = a.getImageData(b, c, d, e);
        } catch (j) {
          return !1;
        }
        var k = this.blurX / 2;
        if (isNaN(k) || 0 > k) return !1;
        k |= 0;
        var l = this.blurY / 2;
        if (isNaN(l) || 0 > l) return !1;
        if (((l |= 0), 0 == k && 0 == l)) return !1;
        var m = this.quality;
        (isNaN(m) || 1 > m) && (m = 1),
          (m |= 0),
          m > 3 && (m = 3),
          1 > m && (m = 1);
        var b,
          c,
          n,
          o,
          p,
          q,
          r,
          s,
          t,
          u,
          v,
          w,
          x,
          y,
          z,
          A = i.data,
          B = k + k + 1,
          C = l + l + 1,
          D = d - 1,
          E = e - 1,
          F = k + 1,
          G = l + 1,
          H = { r: 0, b: 0, g: 0, a: 0, next: null },
          I = H;
        for (n = 1; B > n; n++)
          I = I.next = { r: 0, b: 0, g: 0, a: 0, next: null };
        I.next = H;
        var J = { r: 0, b: 0, g: 0, a: 0, next: null },
          K = J;
        for (n = 1; C > n; n++)
          K = K.next = { r: 0, b: 0, g: 0, a: 0, next: null };
        K.next = J;
        for (var L = null; m-- > 0; ) {
          r = q = 0;
          var M = this.mul_table[k],
            N = this.shg_table[k];
          for (c = e; --c > -1; ) {
            for (
              s = F * (w = A[q]),
                t = F * (x = A[q + 1]),
                u = F * (y = A[q + 2]),
                v = F * (z = A[q + 3]),
                I = H,
                n = F;
              --n > -1;

            )
              (I.r = w), (I.g = x), (I.b = y), (I.a = z), (I = I.next);
            for (n = 1; F > n; n++)
              (o = q + ((n > D ? D : n) << 2)),
                (s += I.r = A[o]),
                (t += I.g = A[o + 1]),
                (u += I.b = A[o + 2]),
                (v += I.a = A[o + 3]),
                (I = I.next);
            for (L = H, b = 0; d > b; b++)
              (A[q++] = (s * M) >>> N),
                (A[q++] = (t * M) >>> N),
                (A[q++] = (u * M) >>> N),
                (A[q++] = (v * M) >>> N),
                (o = (r + ((o = b + k + 1) < D ? o : D)) << 2),
                (s -= L.r - (L.r = A[o])),
                (t -= L.g - (L.g = A[o + 1])),
                (u -= L.b - (L.b = A[o + 2])),
                (v -= L.a - (L.a = A[o + 3])),
                (L = L.next);
            r += d;
          }
          for (
            M = this.mul_table[l], N = this.shg_table[l], b = 0;
            d > b;
            b++
          ) {
            for (
              q = b << 2,
                s = G * (w = A[q]),
                t = G * (x = A[q + 1]),
                u = G * (y = A[q + 2]),
                v = G * (z = A[q + 3]),
                K = J,
                n = 0;
              G > n;
              n++
            )
              (K.r = w), (K.g = x), (K.b = y), (K.a = z), (K = K.next);
            for (p = d, n = 1; l >= n; n++)
              (q = (p + b) << 2),
                (s += K.r = A[q]),
                (t += K.g = A[q + 1]),
                (u += K.b = A[q + 2]),
                (v += K.a = A[q + 3]),
                (K = K.next),
                E > n && (p += d);
            if (((q = b), (L = J), m > 0))
              for (c = 0; e > c; c++)
                (o = q << 2),
                  (A[o + 3] = z = (v * M) >>> N),
                  z > 0
                    ? ((A[o] = (s * M) >>> N),
                      (A[o + 1] = (t * M) >>> N),
                      (A[o + 2] = (u * M) >>> N))
                    : (A[o] = A[o + 1] = A[o + 2] = 0),
                  (o = (b + ((o = c + G) < E ? o : E) * d) << 2),
                  (s -= L.r - (L.r = A[o])),
                  (t -= L.g - (L.g = A[o + 1])),
                  (u -= L.b - (L.b = A[o + 2])),
                  (v -= L.a - (L.a = A[o + 3])),
                  (L = L.next),
                  (q += d);
            else
              for (c = 0; e > c; c++)
                (o = q << 2),
                  (A[o + 3] = z = (v * M) >>> N),
                  z > 0
                    ? ((z = 255 / z),
                      (A[o] = ((s * M) >>> N) * z),
                      (A[o + 1] = ((t * M) >>> N) * z),
                      (A[o + 2] = ((u * M) >>> N) * z))
                    : (A[o] = A[o + 1] = A[o + 2] = 0),
                  (o = (b + ((o = c + G) < E ? o : E) * d) << 2),
                  (s -= L.r - (L.r = A[o])),
                  (t -= L.g - (L.g = A[o + 1])),
                  (u -= L.b - (L.b = A[o + 2])),
                  (v -= L.a - (L.a = A[o + 3])),
                  (L = L.next),
                  (q += d);
          }
        }
        return f.putImageData(i, g, h), !0;
      }),
      (b.clone = function () {
        return new a(this.blurX, this.blurY, this.quality);
      }),
      (b.toString = function () {
        return "[BlurFilter]";
      }),
      (createjs.BlurFilter = a);
  })(),
  (this.createjs = this.createjs || {}),
  (function () {
    "use strict";
    var a = function (a) {
        this.initialize(a);
      },
      b = (a.prototype = new createjs.Filter());
    (b.initialize = function (a) {
      this.alphaMap = a;
    }),
      (b.alphaMap = null),
      (b._alphaMap = null),
      (b._mapData = null),
      (b.applyFilter = function (a, b, c, d, e, f, g, h) {
        if (!this.alphaMap) return !0;
        if (!this._prepAlphaMap()) return !1;
        (f = f || a), null == g && (g = b), null == h && (h = c);
        try {
          var i = a.getImageData(b, c, d, e);
        } catch (j) {
          return !1;
        }
        for (
          var k = i.data, l = this._mapData, m = k.length, n = 0;
          m > n;
          n += 4
        )
          k[n + 3] = l[n] || 0;
        return f.putImageData(i, g, h), !0;
      }),
      (b.clone = function () {
        return new a(this.alphaMap);
      }),
      (b.toString = function () {
        return "[AlphaMapFilter]";
      }),
      (b._prepAlphaMap = function () {
        if (!this.alphaMap) return !1;
        if (this.alphaMap == this._alphaMap && this._mapData) return !0;
        this._mapData = null;
        var a,
          b = (this._alphaMap = this.alphaMap),
          c = b;
        b instanceof HTMLCanvasElement
          ? (a = c.getContext("2d"))
          : ((c = createjs.createCanvas
              ? createjs.createCanvas()
              : document.createElement("canvas")),
            (c.width = b.width),
            (c.height = b.height),
            (a = c.getContext("2d")),
            a.drawImage(b, 0, 0));
        try {
          var d = a.getImageData(0, 0, b.width, b.height);
        } catch (e) {
          return !1;
        }
        return (this._mapData = d.data), !0;
      }),
      (createjs.AlphaMapFilter = a);
  })(),
  (this.createjs = this.createjs || {}),
  (function () {
    "use strict";
    var a = function (a) {
        this.initialize(a);
      },
      b = (a.prototype = new createjs.Filter());
    (b.initialize = function (a) {
      this.mask = a;
    }),
      (b.mask = null),
      (b.applyFilter = function (a, b, c, d, e, f, g, h) {
        return this.mask
          ? ((f = f || a),
            null == g && (g = b),
            null == h && (h = c),
            f.save(),
            (f.globalCompositeOperation = "destination-in"),
            f.drawImage(this.mask, g, h),
            f.restore(),
            !0)
          : !0;
      }),
      (b.clone = function () {
        return new a(this.mask);
      }),
      (b.toString = function () {
        return "[AlphaMaskFilter]";
      }),
      (createjs.AlphaMaskFilter = a);
  })(),
  (this.createjs = this.createjs || {}),
  (function () {
    "use strict";
    var a = function (a, b, c, d, e, f, g, h) {
        this.initialize(a, b, c, d, e, f, g, h);
      },
      b = (a.prototype = new createjs.Filter());
    (b.redMultiplier = 1),
      (b.greenMultiplier = 1),
      (b.blueMultiplier = 1),
      (b.alphaMultiplier = 1),
      (b.redOffset = 0),
      (b.greenOffset = 0),
      (b.blueOffset = 0),
      (b.alphaOffset = 0),
      (b.initialize = function (a, b, c, d, e, f, g, h) {
        (this.redMultiplier = null != a ? a : 1),
          (this.greenMultiplier = null != b ? b : 1),
          (this.blueMultiplier = null != c ? c : 1),
          (this.alphaMultiplier = null != d ? d : 1),
          (this.redOffset = e || 0),
          (this.greenOffset = f || 0),
          (this.blueOffset = g || 0),
          (this.alphaOffset = h || 0);
      }),
      (b.applyFilter = function (a, b, c, d, e, f, g, h) {
        (f = f || a), null == g && (g = b), null == h && (h = c);
        try {
          var i = a.getImageData(b, c, d, e);
        } catch (j) {
          return !1;
        }
        for (var k = i.data, l = k.length, m = 0; l > m; m += 4)
          (k[m] = k[m] * this.redMultiplier + this.redOffset),
            (k[m + 1] = k[m + 1] * this.greenMultiplier + this.greenOffset),
            (k[m + 2] = k[m + 2] * this.blueMultiplier + this.blueOffset),
            (k[m + 3] = k[m + 3] * this.alphaMultiplier + this.alphaOffset);
        return f.putImageData(i, g, h), !0;
      }),
      (b.toString = function () {
        return "[ColorFilter]";
      }),
      (b.clone = function () {
        return new a(
          this.redMultiplier,
          this.greenMultiplier,
          this.blueMultiplier,
          this.alphaMultiplier,
          this.redOffset,
          this.greenOffset,
          this.blueOffset,
          this.alphaOffset
        );
      }),
      (createjs.ColorFilter = a);
  })(),
  (this.createjs = this.createjs || {}),
  (function () {
    "use strict";
    var a = function (a, b, c, d) {
        this.initialize(a, b, c, d);
      },
      b = a.prototype;
    (a.DELTA_INDEX = [
      0,
      0.01,
      0.02,
      0.04,
      0.05,
      0.06,
      0.07,
      0.08,
      0.1,
      0.11,
      0.12,
      0.14,
      0.15,
      0.16,
      0.17,
      0.18,
      0.2,
      0.21,
      0.22,
      0.24,
      0.25,
      0.27,
      0.28,
      0.3,
      0.32,
      0.34,
      0.36,
      0.38,
      0.4,
      0.42,
      0.44,
      0.46,
      0.48,
      0.5,
      0.53,
      0.56,
      0.59,
      0.62,
      0.65,
      0.68,
      0.71,
      0.74,
      0.77,
      0.8,
      0.83,
      0.86,
      0.89,
      0.92,
      0.95,
      0.98,
      1,
      1.06,
      1.12,
      1.18,
      1.24,
      1.3,
      1.36,
      1.42,
      1.48,
      1.54,
      1.6,
      1.66,
      1.72,
      1.78,
      1.84,
      1.9,
      1.96,
      2,
      2.12,
      2.25,
      2.37,
      2.5,
      2.62,
      2.75,
      2.87,
      3,
      3.2,
      3.4,
      3.6,
      3.8,
      4,
      4.3,
      4.7,
      4.9,
      5,
      5.5,
      6,
      6.5,
      6.8,
      7,
      7.3,
      7.5,
      7.8,
      8,
      8.4,
      8.7,
      9,
      9.4,
      9.6,
      9.8,
      10,
    ]),
      (a.IDENTITY_MATRIX = [
        1,
        0,
        0,
        0,
        0,
        0,
        1,
        0,
        0,
        0,
        0,
        0,
        1,
        0,
        0,
        0,
        0,
        0,
        1,
        0,
        0,
        0,
        0,
        0,
        1,
      ]),
      (a.LENGTH = a.IDENTITY_MATRIX.length),
      (b.initialize = function (a, b, c, d) {
        return this.reset(), this.adjustColor(a, b, c, d), this;
      }),
      (b.reset = function () {
        return this.copyMatrix(a.IDENTITY_MATRIX);
      }),
      (b.adjustColor = function (a, b, c, d) {
        return (
          this.adjustHue(d),
          this.adjustContrast(b),
          this.adjustBrightness(a),
          this.adjustSaturation(c)
        );
      }),
      (b.adjustBrightness = function (a) {
        return 0 == a || isNaN(a)
          ? this
          : ((a = this._cleanValue(a, 255)),
            this._multiplyMatrix([
              1,
              0,
              0,
              0,
              a,
              0,
              1,
              0,
              0,
              a,
              0,
              0,
              1,
              0,
              a,
              0,
              0,
              0,
              1,
              0,
              0,
              0,
              0,
              0,
              1,
            ]),
            this);
      }),
      (b.adjustContrast = function (b) {
        if (0 == b || isNaN(b)) return this;
        b = this._cleanValue(b, 100);
        var c;
        return (
          0 > b
            ? (c = 127 + 127 * (b / 100))
            : ((c = b % 1),
              (c =
                0 == c
                  ? a.DELTA_INDEX[b]
                  : a.DELTA_INDEX[b << 0] * (1 - c) +
                    a.DELTA_INDEX[(b << 0) + 1] * c),
              (c = 127 * c + 127)),
          this._multiplyMatrix([
            c / 127,
            0,
            0,
            0,
            0.5 * (127 - c),
            0,
            c / 127,
            0,
            0,
            0.5 * (127 - c),
            0,
            0,
            c / 127,
            0,
            0.5 * (127 - c),
            0,
            0,
            0,
            1,
            0,
            0,
            0,
            0,
            0,
            1,
          ]),
          this
        );
      }),
      (b.adjustSaturation = function (a) {
        if (0 == a || isNaN(a)) return this;
        a = this._cleanValue(a, 100);
        var b = 1 + (a > 0 ? (3 * a) / 100 : a / 100),
          c = 0.3086,
          d = 0.6094,
          e = 0.082;
        return (
          this._multiplyMatrix([
            c * (1 - b) + b,
            d * (1 - b),
            e * (1 - b),
            0,
            0,
            c * (1 - b),
            d * (1 - b) + b,
            e * (1 - b),
            0,
            0,
            c * (1 - b),
            d * (1 - b),
            e * (1 - b) + b,
            0,
            0,
            0,
            0,
            0,
            1,
            0,
            0,
            0,
            0,
            0,
            1,
          ]),
          this
        );
      }),
      (b.adjustHue = function (a) {
        if (0 == a || isNaN(a)) return this;
        a = (this._cleanValue(a, 180) / 180) * Math.PI;
        var b = Math.cos(a),
          c = Math.sin(a),
          d = 0.213,
          e = 0.715,
          f = 0.072;
        return (
          this._multiplyMatrix([
            d + b * (1 - d) + c * -d,
            e + b * -e + c * -e,
            f + b * -f + c * (1 - f),
            0,
            0,
            d + b * -d + 0.143 * c,
            e + b * (1 - e) + 0.14 * c,
            f + b * -f + c * -0.283,
            0,
            0,
            d + b * -d + c * -(1 - d),
            e + b * -e + c * e,
            f + b * (1 - f) + c * f,
            0,
            0,
            0,
            0,
            0,
            1,
            0,
            0,
            0,
            0,
            0,
            1,
          ]),
          this
        );
      }),
      (b.concat = function (b) {
        return (
          (b = this._fixMatrix(b)),
          b.length != a.LENGTH ? this : (this._multiplyMatrix(b), this)
        );
      }),
      (b.clone = function () {
        return new a().copyMatrix(this);
      }),
      (b.toArray = function () {
        for (var b = [], c = 0, d = a.LENGTH; d > c; c++) b[c] = this[c];
        return b;
      }),
      (b.copyMatrix = function (b) {
        for (var c = a.LENGTH, d = 0; c > d; d++) this[d] = b[d];
        return this;
      }),
      (b.toString = function () {
        return "[ColorMatrix]";
      }),
      (b._multiplyMatrix = function (a) {
        for (var b = [], c = 0; 5 > c; c++) {
          for (var d = 0; 5 > d; d++) b[d] = this[d + 5 * c];
          for (var d = 0; 5 > d; d++) {
            for (var e = 0, f = 0; 5 > f; f++) e += a[d + 5 * f] * b[f];
            this[d + 5 * c] = e;
          }
        }
      }),
      (b._cleanValue = function (a, b) {
        return Math.min(b, Math.max(-b, a));
      }),
      (b._fixMatrix = function (b) {
        return (
          b instanceof a && (b = b.toArray()),
          b.length < a.LENGTH
            ? (b = b
                .slice(0, b.length)
                .concat(a.IDENTITY_MATRIX.slice(b.length, a.LENGTH)))
            : b.length > a.LENGTH && (b = b.slice(0, a.LENGTH)),
          b
        );
      }),
      (createjs.ColorMatrix = a);
  })(),
  (this.createjs = this.createjs || {}),
  (function () {
    "use strict";
    var a = function (a) {
        this.initialize(a);
      },
      b = (a.prototype = new createjs.Filter());
    (b.matrix = null),
      (b.initialize = function (a) {
        this.matrix = a;
      }),
      (b.applyFilter = function (a, b, c, d, e, f, g, h) {
        (f = f || a), null == g && (g = b), null == h && (h = c);
        try {
          var i = a.getImageData(b, c, d, e);
        } catch (j) {
          return !1;
        }
        for (
          var k,
            l,
            m,
            n,
            o = i.data,
            p = o.length,
            q = this.matrix,
            r = q[0],
            s = q[1],
            t = q[2],
            u = q[3],
            v = q[4],
            w = q[5],
            x = q[6],
            y = q[7],
            z = q[8],
            A = q[9],
            B = q[10],
            C = q[11],
            D = q[12],
            E = q[13],
            F = q[14],
            G = q[15],
            H = q[16],
            I = q[17],
            J = q[18],
            K = q[19],
            L = 0;
          p > L;
          L += 4
        )
          (k = o[L]),
            (l = o[L + 1]),
            (m = o[L + 2]),
            (n = o[L + 3]),
            (o[L] = k * r + l * s + m * t + n * u + v),
            (o[L + 1] = k * w + l * x + m * y + n * z + A),
            (o[L + 2] = k * B + l * C + m * D + n * E + F),
            (o[L + 3] = k * G + l * H + m * I + n * J + K);
        return f.putImageData(i, g, h), !0;
      }),
      (b.toString = function () {
        return "[ColorMatrixFilter]";
      }),
      (b.clone = function () {
        return new a(this.matrix);
      }),
      (createjs.ColorMatrixFilter = a);
  })(),
  (this.createjs = this.createjs || {}),
  (function () {
    "use strict";
    var a = function () {
      throw "Touch cannot be instantiated";
    };
    (a.isSupported = function () {
      return (
        "ontouchstart" in window ||
        (window.navigator.msPointerEnabled &&
          window.navigator.msMaxTouchPoints > 0) ||
        (window.navigator.pointerEnabled && window.navigator.maxTouchPoints > 0)
      );
    }),
      (a.enable = function (b, c, d) {
        return b && b.canvas && a.isSupported()
          ? ((b.__touch = {
              pointers: {},
              multitouch: !c,
              preventDefault: !d,
              count: 0,
            }),
            "ontouchstart" in window
              ? a._IOS_enable(b)
              : (window.navigator.msPointerEnabled ||
                  window.navigator.pointerEnabled) &&
                a._IE_enable(b),
            !0)
          : !1;
      }),
      (a.disable = function (b) {
        b &&
          ("ontouchstart" in window
            ? a._IOS_disable(b)
            : (window.navigator.msPointerEnabled ||
                window.navigator.pointerEnabled) &&
              a._IE_disable(b));
      }),
      (a._IOS_enable = function (b) {
        var c = b.canvas,
          d = (b.__touch.f = function (c) {
            a._IOS_handleEvent(b, c);
          });
        c.addEventListener("touchstart", d, !1),
          c.addEventListener("touchmove", d, !1),
          c.addEventListener("touchend", d, !1),
          c.addEventListener("touchcancel", d, !1);
      }),
      (a._IOS_disable = function (a) {
        var b = a.canvas;
        if (b) {
          var c = a.__touch.f;
          b.removeEventListener("touchstart", c, !1),
            b.removeEventListener("touchmove", c, !1),
            b.removeEventListener("touchend", c, !1),
            b.removeEventListener("touchcancel", c, !1);
        }
      }),
      (a._IOS_handleEvent = function (a, b) {
        if (a) {
          a.__touch.preventDefault && b.preventDefault && b.preventDefault();
          for (
            var c = b.changedTouches, d = b.type, e = 0, f = c.length;
            f > e;
            e++
          ) {
            var g = c[e],
              h = g.identifier;
            g.target == a.canvas &&
              ("touchstart" == d
                ? this._handleStart(a, h, b, g.pageX, g.pageY)
                : "touchmove" == d
                ? this._handleMove(a, h, b, g.pageX, g.pageY)
                : ("touchend" == d || "touchcancel" == d) &&
                  this._handleEnd(a, h, b));
          }
        }
      }),
      (a._IE_enable = function (b) {
        var c = b.canvas,
          d = (b.__touch.f = function (c) {
            a._IE_handleEvent(b, c);
          });
        void 0 === window.navigator.pointerEnabled
          ? (c.addEventListener("MSPointerDown", d, !1),
            window.addEventListener("MSPointerMove", d, !1),
            window.addEventListener("MSPointerUp", d, !1),
            window.addEventListener("MSPointerCancel", d, !1),
            b.__touch.preventDefault && (c.style.msTouchAction = "none"))
          : (c.addEventListener("pointerdown", d, !1),
            window.addEventListener("pointermove", d, !1),
            window.addEventListener("pointerup", d, !1),
            window.addEventListener("pointercancel", d, !1),
            b.__touch.preventDefault && (c.style.touchAction = "none")),
          (b.__touch.activeIDs = {});
      }),
      (a._IE_disable = function (a) {
        var b = a.__touch.f;
        void 0 === window.navigator.pointerEnabled
          ? (window.removeEventListener("MSPointerMove", b, !1),
            window.removeEventListener("MSPointerUp", b, !1),
            window.removeEventListener("MSPointerCancel", b, !1),
            a.canvas && a.canvas.removeEventListener("MSPointerDown", b, !1))
          : (window.removeEventListener("pointermove", b, !1),
            window.removeEventListener("pointerup", b, !1),
            window.removeEventListener("pointercancel", b, !1),
            a.canvas && a.canvas.removeEventListener("pointerdown", b, !1));
      }),
      (a._IE_handleEvent = function (a, b) {
        if (a) {
          a.__touch.preventDefault && b.preventDefault && b.preventDefault();
          var c = b.type,
            d = b.pointerId,
            e = a.__touch.activeIDs;
          if ("MSPointerDown" == c || "pointerdown" == c) {
            if (b.srcElement != a.canvas) return;
            (e[d] = !0), this._handleStart(a, d, b, b.pageX, b.pageY);
          } else
            e[d] &&
              ("MSPointerMove" == c || "pointermove" == c
                ? this._handleMove(a, d, b, b.pageX, b.pageY)
                : ("MSPointerUp" == c ||
                    "MSPointerCancel" == c ||
                    "pointerup" == c ||
                    "pointercancel" == c) &&
                  (delete e[d], this._handleEnd(a, d, b)));
        }
      }),
      (a._handleStart = function (a, b, c, d, e) {
        var f = a.__touch;
        if (f.multitouch || !f.count) {
          var g = f.pointers;
          g[b] || ((g[b] = !0), f.count++, a._handlePointerDown(b, c, d, e));
        }
      }),
      (a._handleMove = function (a, b, c, d, e) {
        a.__touch.pointers[b] && a._handlePointerMove(b, c, d, e);
      }),
      (a._handleEnd = function (a, b, c) {
        var d = a.__touch,
          e = d.pointers;
        e[b] && (d.count--, a._handlePointerUp(b, c, !0), delete e[b]);
      }),
      (createjs.Touch = a);
  })(),
  (this.createjs = this.createjs || {}),
  (function () {
    "use strict";
    var a = (createjs.EaselJS = createjs.EaselJS || {});
    (a.version = "0.7.1"), (a.buildDate = "Thu, 12 Dec 2013 23:33:39 GMT");
  })();
