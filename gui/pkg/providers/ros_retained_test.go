package providers

import (
	"testing"
	"time"

	"github.com/mowglinext/mowglinext/pkg/foxglove"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

// newTestRosProvider builds a provider around a foxglove client that is never
// connected: Subscribe/Unsubscribe only update local bookkeeping.
func newTestRosProvider() *RosProvider {
	return &RosProvider{
		client:             foxglove.NewClient("ws://127.0.0.1:1"),
		subscribers:        make(map[string]map[string]*RosSubscriber),
		lastMessage:        make(map[string][]byte),
		foxgloveSubscribed: make(map[string]bool),
	}
}

func waitForMessage(t *testing.T, ch <-chan []byte) []byte {
	t.Helper()
	select {
	case msg := <-ch:
		return msg
	case <-time.After(2 * time.Second):
		t.Fatal("timed out waiting for replayed message")
		return nil
	}
}

// mowProgress is published by map_server_node only when coverage changes, so
// its upstream subscription and cached last message must survive the last
// browser leaving; otherwise a re-opened map page has no mowed overlay.
func TestRetainedTopicKeepsUpstreamSubscriptionAndCache(t *testing.T) {
	require.True(t, retainedTopics["mowProgress"])
	r := newTestRosProvider()

	require.NoError(t, r.Subscribe("mowProgress", "tab-1", 0, func([]byte) {}))
	r.fanOut("mowProgress", []byte(`{"data":[0,100]}`))
	r.UnSubscribe("mowProgress", "tab-1")

	r.mtx.Lock()
	assert.True(t, r.foxgloveSubscribed["mowProgress"], "upstream subscription dropped")
	assert.Equal(t, `{"data":[0,100]}`, string(r.lastMessage["mowProgress"]), "cache dropped")
	r.mtx.Unlock()

	got := make(chan []byte, 1)
	require.NoError(t, r.Subscribe("mowProgress", "tab-2", 0, func(msg []byte) { got <- msg }))
	defer r.UnSubscribe("mowProgress", "tab-2")
	assert.Equal(t, `{"data":[0,100]}`, string(waitForMessage(t, got)))
}

// Non-retained topics keep the lazy policy: no listener, no upstream
// subscription, no stale cache.
func TestNonRetainedTopicIsDroppedWithItsLastListener(t *testing.T) {
	require.False(t, retainedTopics["lidar"])
	r := newTestRosProvider()

	require.NoError(t, r.Subscribe("lidar", "tab-1", 0, func([]byte) {}))
	r.fanOut("lidar", []byte(`{"ranges":[1]}`))
	r.UnSubscribe("lidar", "tab-1")

	r.mtx.Lock()
	defer r.mtx.Unlock()
	assert.False(t, r.foxgloveSubscribed["lidar"])
	_, cached := r.lastMessage["lidar"]
	assert.False(t, cached)
}

func TestInitRetainedSubscriptionsSubscribesUpstreamWithoutListeners(t *testing.T) {
	r := newTestRosProvider()
	r.initRetainedSubscriptions()

	r.mtx.Lock()
	defer r.mtx.Unlock()
	for key := range retainedTopics {
		assert.True(t, r.foxgloveSubscribed[key], key)
	}
}
