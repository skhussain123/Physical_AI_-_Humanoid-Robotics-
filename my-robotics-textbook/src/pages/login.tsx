import React, { useState } from 'react';
import clsx from 'clsx';
import Layout from '@theme/Layout';

const LoginPage = () => {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    // In a real app, this would handle authentication
    console.log('Login attempt with:', { email, password });
  };

  return (
    <Layout title="Login" description="Login to your robotics textbook account">
      <main className="container margin-vert--lg">
        <div className="row">
          <div className="col col--6 col--offset-3">
            <div className="padding--lg text--center">
              <h1>Login to Robotics Textbook</h1>
              <p>Continue your journey in Physical AI & Humanoid Robotics</p>
            </div>

            <div className="padding-horiz--md">
              <form onSubmit={handleSubmit}>
                <div className="margin-bottom--lg">
                  <label htmlFor="email" className="form-label">Email</label>
                  <input
                    type="email"
                    id="email"
                    className="form-control"
                    value={email}
                    onChange={(e) => setEmail(e.target.value)}
                    required
                  />
                </div>

                <div className="margin-bottom--lg">
                  <label htmlFor="password" className="form-label">Password</label>
                  <input
                    type="password"
                    id="password"
                    className="form-control"
                    value={password}
                    onChange={(e) => setPassword(e.target.value)}
                    required
                  />
                </div>

                <div className="margin-bottom--lg">
                  <button type="submit" className="button button--primary button--block">
                    Sign In
                  </button>
                </div>

                <div className="text--center">
                  <p>
                    Don't have an account? <a href="/signup">Sign up</a>
                  </p>
                  <p>
                    <a href="/">Back to Home</a>
                  </p>
                </div>
              </form>
            </div>
          </div>
        </div>
      </main>
    </Layout>
  );
};

export default LoginPage;