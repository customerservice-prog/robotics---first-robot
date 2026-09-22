from app.services.memory import MemoryStore


def test_memory_round_trip(tmp_path):
    store = MemoryStore(str(tmp_path / "memory.db"))
    created = store.add("White resin chairs are in aisle 3", ["warehouse"], 8)
    assert created.id > 0
    found = store.search("where are resin chairs")
    assert found
    assert "aisle 3" in found[0].content


def test_delete_memory(tmp_path):
    store = MemoryStore(str(tmp_path / "memory.db"))
    created = store.add("test fact")
    assert store.delete(created.id) is True
    assert store.delete(created.id) is False
