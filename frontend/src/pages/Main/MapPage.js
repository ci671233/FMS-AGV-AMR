import React, { useState, useEffect } from 'react';
import axios from 'axios';

function MapPage() {
    const [maps, setMaps] = useState([]);
    const [selectedMap, setSelectedMap] = useState(null);
    const [name, setName] = useState('');
    const [file, setFile] = useState(null);
    const [meta, setMeta] = useState('');

    useEffect(() => {
        const fetchMaps = async () => {
            try {
                const response = await axios.get('http://localhost:5557/map');
                setMaps(response.data);
            } catch (error) {
                console.error('Error fetching maps:', error);
            }
        };

        fetchMaps();
    }, []);

    const handleMapSelect = (map) => {
        setSelectedMap(map);
    };

    const handleFileChange = (e) => {
        setFile(e.target.files[0]);
    };

    const handleUpload = async (e) => {
        e.preventDefault();
        const formData = new FormData();
        formData.append('file', file);
        formData.append('name', name);
        formData.append('meta', JSON.stringify({ additionalInfo: meta }));

        try {
            await axios.post('http://localhost:5557/map/upload', formData, {
                headers: { 'Authorization': `Bearer ${localStorage.getItem('token')}` }
            });
            alert('Map uploaded successfully');
            const response = await axios.get('http://localhost:5557/map');
            setMaps(response.data);
        } catch (error) {
            console.error('Error uploading map:', error);
        }
    };

    return (
        <div>
            <h2>Map Editor</h2>
            <ul>
                {maps.map(map => (
                    <li key={map._id} onClick={() => handleMapSelect(map)}>
                        {map.name}
                    </li>
                ))}
            </ul>
            {selectedMap && (
                <div>
                    <h3>Edit Map: {selectedMap.name}</h3>
                </div>
            )}
            <form onSubmit={handleUpload}>
                <input
                    type="text"
                    placeholder="Map Name"
                    value={name}
                    onChange={(e) => setName(e.target.value)}
                    required
                />
                <input
                    type="file"
                    accept=".pgm, .yaml"
                    onChange={handleFileChange}
                    required
                />
                <textarea
                    placeholder="Meta Information (JSON format)"
                    value={meta}
                    onChange={(e) => setMeta(e.target.value)}
                    required
                />
                <button type="submit">Upload Map</button>
            </form>
        </div>
    );
}

export default MapPage;

