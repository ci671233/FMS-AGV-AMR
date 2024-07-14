import React, { useState, useEffect } from 'react';
import axios from 'axios';

function MapPage() {
    const [maps, setMaps] = useState([]);
    const [selectedMap, setSelectedMap] = useState(null);

    useEffect(() => {
        const fetchMaps = async () => {
            try {
                const response = await axios.get('http://localhost:5557/maps');
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

    const handleSaveMap = async () => {
        // 맵 저장 로직
    };

    return (
        <div>
            <h2>Map Editor</h2>
            <ul>
                {maps.map(map => (
                    <li key={map.id} onClick={() => handleMapSelect(map)}>
                        {map.name}
                    </li>
                ))}
            </ul>
            {selectedMap && (
                <div>
                    <h3>Edit Map: {selectedMap.name}</h3>
                    {/* 맵 편집 로직 */}
                    <button onClick={handleSaveMap}>Save Map</button>
                </div>
            )}
        </div>
    );
}

export default MapPage;

