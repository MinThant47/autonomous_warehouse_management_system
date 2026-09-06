import { useEffect, useState } from "react";
import API from "../api";

function TaskForm() {
  const [outboundSerial, setOutboundSerial] = useState("");
  const [gate, setGate] = useState("Gate_3");
  const [pickupShelf, setPickupShelf] = useState("");
  const [destinationShelf, setDestinationShelf] = useState("");
  const [shelves, setShelves] = useState([]);
  const [inventoryItems, setInventoryItems] = useState([]);
  const [message, setMessage] = useState("");

  useEffect(() => {
    Promise.all([API.get("/warehouse/shelves"), API.get("/warehouse/inventory")])
      .then(([shelvesResponse, inventoryResponse]) => {
        setShelves(shelvesResponse.data);
        setInventoryItems(inventoryResponse.data);
      })
      .catch(() => setMessage("Unable to load warehouse shelves"));
  }, []);

  const submit = async (route, payload) => {
    try {
      const { data } = await API.post(route, payload);
      setMessage(data.message);
      const [shelvesResponse, inventoryResponse] = await Promise.all([
        API.get("/warehouse/shelves"),
        API.get("/warehouse/inventory"),
      ]);
      setShelves(shelvesResponse.data);
      setInventoryItems(inventoryResponse.data);
    } catch (error) {
      setMessage(error.response?.data?.error || "Server error");
    }
  };

  const shelfOptions = (empty) => shelves.filter((shelf) => shelf.empty === empty);
  const destinationOptions = shelfOptions(1);

  return (
    <div className="task-box">
      <h2>Create Warehouse Task</h2>

      <section className="task-section">
        <h3>Outbound task</h3>
        <p>The selected item&apos;s current shelf is found automatically from inventory.</p>
        <label htmlFor="outbound-serial">Item</label>
        <select id="outbound-serial" value={outboundSerial} onChange={(e) => setOutboundSerial(e.target.value)}>
          <option value="">Select an item</option>
          {inventoryItems.map((item) => <option key={item.item_id} value={item.item_id}>{item.item_name} ({item.item_id})</option>)}
        </select>
        <label htmlFor="outbound-gate">Outbound gate</label>
        <select id="outbound-gate" value={gate} onChange={(e) => setGate(e.target.value)}>
          <option value="Gate_3">Gate_3</option>
          <option value="Gate_4">Gate_4</option>
        </select>
        <button onClick={() => submit("/tasks/outbound", { serial_number: outboundSerial, gate })}>Create outbound task</button>
      </section>

      <section className="task-section">
        <h3>Relocation task</h3>
        <p>The item is identified from the selected occupied shelf and can be moved to any available shelf.</p>
        <label htmlFor="pickup-shelf">Current shelf</label>
        <select id="pickup-shelf" value={pickupShelf} onChange={(e) => setPickupShelf(e.target.value)}>
          <option value="">Select occupied shelf</option>
          {shelfOptions(0).map((shelf) => <option key={shelf.shelf_id} value={shelf.shelf_id}>{shelf.shelf_id}</option>)}
        </select>
        <label htmlFor="destination-shelf">Destination shelf</label>
        <select id="destination-shelf" value={destinationShelf} onChange={(e) => setDestinationShelf(e.target.value)}>
          <option value="">Select available shelf</option>
          {destinationOptions.map((shelf) => <option key={shelf.shelf_id} value={shelf.shelf_id}>{shelf.shelf_id}</option>)}
        </select>
        <button onClick={() => submit("/tasks/relocation", { pickup_shelf: pickupShelf, destination_shelf: destinationShelf })}>Create relocation task</button>
      </section>

      {message && <p className="task-message">{message}</p>}
    </div>
  );
}
export default TaskForm;
