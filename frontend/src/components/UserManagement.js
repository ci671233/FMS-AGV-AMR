import React, { useState, useEffect } from 'react';
import axios from 'axios';

function UserManagement() {
  const [pendingAccounts, setPendingAccounts] = useState([]);
  const [error, setError] = useState('');

  useEffect(() => {
    const fetchPendingAccounts = async () => {
      try {
        const response = await axios.get(`${process.env.REACT_APP_API_URL}/admin/pending/account`);
        setPendingAccounts(response.data);
      } catch (err) {
        setError(err.response.data.message);
      }
    };

    fetchPendingAccounts();
  }, []);

  return (
    <div>
      <h2>Account Approval Waiting List</h2>
      {error && <div style={{ color: 'red' }}>{error}</div>}
      <ul>
        {pendingAccounts.map((account) => (
          <li key={account._id}>
            {account.email} - {account.name}
            {/* 승인 버튼을 추가하여 계정을 승인할 수 있도록 합니다. */}
          </li>
        ))}
      </ul>
    </div>
  );
}

export default UserManagement;

